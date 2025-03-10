#include "SerialApi.h"
#include <umsg.h>
#include <mrs_lib/mutex.h>

SerialApi::SerialApi(/* args */)
{
}
SerialApi::SerialApi(std::string dev, int baudrate)
{

    if (!ser.connect(dev, baudrate, false))
    {
        ROS_ERROR("could not open serial port");
        return;
    }
    ser.setBlocking(ser.serial_port_fd_, sizeof(umsg_MessageToTransfer));

    umsg_CRCInit();
}

void SerialApi::calculateDelay(umsg_state_heartbeat_t heartbeat)
{
    // auto curr_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
    auto curr_time = ros::Time::now();
    auto [start_time, sequential] = mrs_lib::get_mutexed(mutex_sync_time, sync_time_ROS_send, sequence_number);

    ros::Duration diff;
    diff.fromNSec((curr_time - start_time).toNSec() / 2);

    ros::Time syncTime_R = start_time + diff;
    uint32_t syncTime_F = heartbeat.timestamp_arrived;

    if (heartbeat.seq_num == sequential - 1)
    {
        mrs_lib::set_mutexed(mutex_sync_result, std::make_tuple(syncTime_R, syncTime_F), sync_result);

        double time_difference = static_cast<double>((curr_time - start_time).toNSec()) / 1e6;
        ROS_INFO("[SYNC] curr_time %ld start was %ld delay was %.3f miliseconds", curr_time.toNSec(), start_time.toNSec(), time_difference);
    }
    else
    {
        ROS_ERROR("[SYNC] NOT MATCHING SEQUENCE NUMBERS");
    }
}

uint32_t SerialApi::RosToFcu(const ros::Time &rosTime)
{

    auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
    int64_t diff = (rosTime.toNSec() - syncTime_R.toNSec()) / 1e6;
    int64_t new_stamp = diff + static_cast<int64_t>(syncTime_F);
    return static_cast<uint32_t>(new_stamp);
}

ros::Time SerialApi::FcuToRos(const uint32_t &FcuTime)
{
    auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
    int64_t diff = static_cast<int64_t>(FcuTime) - static_cast<int64_t>(syncTime_F) * 1e6;

    ros::Duration diff_R;
    diff_R.fromNSec(diff);

    return syncTime_R + diff_R;
}

void SerialApi::timerSync(const ros::WallTimerEvent &event)
{

    ROS_INFO_ONCE("[SerialApi]: sync timer spinning");

    auto sequential = mrs_lib::get_mutexed(mutex_sync_time, sequence_number);

    umsg_MessageToTransfer msg;

    msg.s.sync0 = 'M';
    msg.s.sync1 = 'R';
    msg.s.len = UMSG_HEADER_SIZE + sizeof(umsg_state_heartbeat_t) + 1;
    msg.s.state.heartbeat.seq_num = sequential;
    msg.s.state.heartbeat.timestamp_arrived = 0;
    msg.s.msg_class = UMSG_STATE;
    msg.s.msg_type = STATE_HEARTBEAT;
    msg.raw[msg.s.len - 1] = umsg_calcCRC(msg.raw, msg.s.len - 1);

    sequential += 1;

    // auto curr_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
    auto curr_time = ros::Time::now();
    sendPacket(msg);
    // auto curr_time   = ros::time::now();
    mrs_lib::set_mutexed(mutex_sync_time, std::tuple(curr_time, sequential), std::forward_as_tuple(sync_time_ROS_send, sequence_number));

    return;
}

bool SerialApi::isSynced()
{
    return is_synced_;
}
void SerialApi::startReceiver()
{
    recvThread_ = std::thread([this]
                              { this->Receiver(); });
}

void SerialApi::startSyncTimer(ros::NodeHandle &nh_)
{
    timer_sync_ = nh_.createWallTimer(ros::WallDuration(1.00), &SerialApi::timerSync, this);
}

bool SerialApi::waitForPacket(umsg_MessageToTransfer &msg)
{
    ROS_ERROR("[SerialApi]wait for packet not yet implemented!");
}

void SerialApi::sendPacket(umsg_MessageToTransfer &msg)
{
    std::scoped_lock lock(serial_mutex_);
    ser.sendCharArray(msg.raw, msg.s.len);
}
void SerialApi::Receiver()
{
    state = WAITING_FOR_SYNC0;
    bool receptionComplete = false;
    ROS_INFO_ONCE("[SerialApi]: ReceiverActive spinning");
    int readBytes = 0; // amount of read bytes
    int toRead = 0;    // amount of read bytes
    while (true)
    {
        switch (state)
        {
        case WAITING_FOR_SYNC0:
        {

            if (readBytes >= 1)
            {
                if (recvdMsg.s.sync0 != 'M')
                {
                    ROS_ERROR("first is the culprit");
                    goto msg_err;
                }
                state = WAITING_FOR_SYNC1;
                msg_len = 1;
                readBytes = 0;
            }
            else
            {
                toRead = 1;
            }
            break;
        }
        case WAITING_FOR_SYNC1:
        {
            if (readBytes >= 1)
            {
                if (recvdMsg.s.sync1 != 'R')
                {
                    ROS_ERROR("second is the culprit");
                    goto msg_err;
                }
                state = WAITING_FOR_HEADER;
                msg_len = 2;
                readBytes = 0;
            }
            else
            {
                toRead = 1;
            }
            break;
        }
        case WAITING_FOR_HEADER:
        {
            if (readBytes >= UMSG_HEADER_SIZE - msg_len)
            {
                if (recvdMsg.s.len > sizeof(umsg_MessageToTransfer) || msg_len > recvdMsg.s.len)
                {
                    ROS_ERROR("third is the culprit");
                    goto msg_err;
                }
                msg_len += UMSG_HEADER_SIZE - msg_len;
                state = WAITING_FOR_PAYLOAD;
                readBytes = 0;
            }
            else
            {
                toRead = UMSG_HEADER_SIZE - msg_len;
            }
            break;
        }
        // fall through
        case WAITING_FOR_PAYLOAD:
        {
            if (readBytes >= recvdMsg.s.len - msg_len)
            {
                if (umsg_calcCRC(recvdMsg.raw, recvdMsg.s.len - 1) != recvdMsg.raw[recvdMsg.s.len - 1])
                {
                    ROS_ERROR("forth is the culprit");
                    goto msg_err;
                }
                msg_len += recvdMsg.s.len - msg_len;
                receptionComplete = true;
                readBytes = 0;
            }
            else
            {
                toRead = recvdMsg.s.len - msg_len;
            }
        }
        break;

        default:
            goto msg_err;
            break;
        }

        if (receptionComplete)
        {
            if (recvdMsg.s.msg_class == UMSG_STATE && recvdMsg.s.msg_type == STATE_HEARTBEAT)
            {

                umsg_state_heartbeat_t beat = recvdMsg.s.state.heartbeat;

                calculateDelay(beat);
                if (!is_synced_)
                {
                    calculateDelay(beat);
                    is_synced_ = true;
                }
            }
            else
            {
                ROS_ERROR("[SerialApi] receiver synchronization with the functions is not yet implemented");
            }
            // flush
            goto msg_flush;
        }

        toRead += -readBytes;
        if (toRead > 0)
        {

            int received = ser.readSerial(recvdMsg.raw + msg_len + readBytes, toRead);
            readBytes += received;
        }

        continue;
    msg_err:

        // NOTE: another buffer overflow issue here
        ROS_ERROR("message corrupted");
        // flush
    msg_flush:
        msg_len = 0;
        state = WAITING_FOR_SYNC0;
        readBytes = 0;
        toRead = 0;
        receptionComplete = false;
    }
}