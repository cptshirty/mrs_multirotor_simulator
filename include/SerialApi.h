#pragma once
#include <ros/ros.h>
#include "serial_port.h"
#include <umsg_classes.h>
#include <umsg_state.h>
#include <thread>

enum ReceiverState
{
    WAITING_FOR_SYNC0,
    WAITING_FOR_SYNC1,
    WAITING_FOR_HEADER,
    WAITING_FOR_PAYLOAD,
    INVALID_MSG,
};

class SerialApi
{
private:
    std::thread recvThread_;
    void Receiver();
    ros::WallTimer timer_sync_;
    void timerSync(const ros::WallTimerEvent &event);

    std::mutex mutex_sync_time;
    ros::Time sync_time_ROS_send;
    uint32_t sequence_number = 0;
    std::mutex mutex_sync_result;
    std::tuple<ros::Time, uint32_t> sync_result;

    std::mutex serial_mutex_;
    SerialPort ser;
    std::atomic<bool> is_synced_ = false;
    umsg_MessageToTransfer recvdMsg;
    uint32_t msg_len = 0;
    ReceiverState state = WAITING_FOR_SYNC0;
    void calculateDelay(umsg_state_heartbeat_t heartbeat);

public:
    SerialApi();
    SerialApi(std::string dev, int baudrate);
    bool isSynced();
    void startReceiver();
    void startSyncTimer(ros::NodeHandle &nh_);

    bool waitForPacket(umsg_MessageToTransfer &msg);

    void sendPacket(umsg_MessageToTransfer &msg);
    uint32_t RosToFcu(const ros::Time &rosTime);
    ros::Time FcuToRos(const uint32_t &FcuTime);
};