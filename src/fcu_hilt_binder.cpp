/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <rosgraph_msgs/Clock.h>

#include <geometry_msgs/PoseArray.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_multirotor_simulator/uav_system/uav_system.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Float64Srv.h>

#include <mrs_msgs/HwApiActuatorCmd.h>
#include <mrs_msgs/HwApiControlGroupCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgRateCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgCmd.h>
#include <mrs_msgs/HwApiVelocityHdgRateCmd.h>
#include <mrs_msgs/HwApiVelocityHdgCmd.h>
#include <mrs_msgs/HwApiPositionCmd.h>
#include <mrs_msgs/TrackerCommand.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include "serial_port.h"

#include <umsg.h>
#include <umsg_classes.h>


//}

#define GRAV_CONST 9.81

namespace mrs_hitl_binders
{
  enum ReceiverState{
    WAITING_FOR_SYNC0,
    WAITING_FOR_SYNC1,
    WAITING_FOR_HEADER,
    WAITING_FOR_PAYLOAD,
    INVALID_MSG,
};
typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;

/* class MultirotorSimulator //{ */

class FcuBinder : public nodelet::Nodelet {

public:
  virtual void onInit();

private:

  std::mutex serial_mutex_;
  SerialPort ser;
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_;
  std::atomic<bool> is_synced_ = false;
  std::string uav_name;
  umsg_MessageToTransfer recvdMsg;
  uint32_t msg_len = 0;
  ReceiverState state = WAITING_FOR_SYNC0;
  // | ------------------------- params ------------------------- |

  ros::Time  sim_time_;
  std::mutex mutex_sim_time_;

  std::mutex mutex_sync_time;
  ros::Time sync_time_ROS_send;
  uint32_t sequence_number = 0;

  std::mutex mutex_sync_result;
  std::tuple<ros::Time, uint32_t> sync_result;

  int rate_divider;
  
  // | ------------------------- timers ------------------------- |


  // | ------------------------ rtf check ----------------------- |


  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>            ph_actuator_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>        ph_control_group_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeRateCmd>        ph_attitude_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeCmd>            ph_attitude_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> ph_acceleration_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgCmd>     ph_acceleration_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd>     ph_velocity_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd>         ph_velocity_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiPositionCmd>            ph_position_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::TrackerCommand>              ph_tracker_cmd_;
  
  
  // | ----------------------- subscribers ----------------------- |

  mrs_lib::SubscribeHandler<sensor_msgs::Imu>          sh_imu_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>          sh_odom_;
  mrs_lib::SubscribeHandler<sensor_msgs::Range>          sh_rangefinder_;
  mrs_lib::SubscribeHandler<rosgraph_msgs::Clock>        sh_clock_;
  
  void callbackOdometry(const nav_msgs::Odometry::ConstPtr msg);
  void callbackIMU(const  sensor_msgs::Imu::ConstPtr msg);
  void callbackRangeFinder(const  sensor_msgs::Range::ConstPtr msg);
  void callbackTime(const  rosgraph_msgs::Clock::ConstPtr msg);
  // | ------------------------- system ------------------------- |


  // | -------------------------- time -------------------------- |

  
  // | -------------------------- Timers -------------------------- |

  std::thread recvThread_;
  void           Receiver();
  ros::WallTimer timer_sync_;
  void           timerSync(const ros::WallTimerEvent& event);



  // | ------------------------- methods ------------------------ |

  void    calculateDelay(umsg_state_heartbeat_t heartbeat);
  uint32_t RosToFcu(ros::Time &rosTime);
  ros::Time FcuToRos(uint32_t &FcuTime);


  // | --------------- dynamic reconfigure server --------------- |

};

//}

/* onInit() //{ */

void FcuBinder::onInit() {

  is_initialized_ = false;
  rate_divider = 0;
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  if (!(nh_.hasParam("/use_sim_time"))) {
    nh_.setParam("/use_sim_time", true);
  }

  srand(time(NULL));

  mrs_lib::ParamLoader param_loader(nh_, "FcuBinder");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("config_uavs");

  double clock_rate;
  param_loader.loadParam("clock_rate", clock_rate);

  bool sim_time_from_wall_time;
  param_loader.loadParam("sim_time_from_wall_time", sim_time_from_wall_time);


  std::string serial_port;
  param_loader.loadParam("serial_port", serial_port);

  if(!ser.connect(serial_port,2000000,false)){
    ROS_ERROR("could not open serial port");
    return;
  }
  ser.setBlocking(ser.serial_port_fd_,1);
  
  std::string uav_name;

  param_loader.loadParam("uav_name", uav_name);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[FcuBinder]: could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |
  ph_actuator_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>(nh_,"actuators_cmd",1,false);
  //ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>(nh_,uav_name + "/control_group_cmd",1,false);

  // | ----------------------- subscribers ----------------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = uav_name;
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();


  sh_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts,"imu",&FcuBinder::callbackIMU,this);
  sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts,"odom",&FcuBinder::callbackOdometry,this);
  sh_rangefinder_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts,"rangefinder",&FcuBinder::callbackRangeFinder,this);
  sh_clock_ = mrs_lib::SubscribeHandler<rosgraph_msgs::Clock>(shopts, "clock_in", &FcuBinder::callbackTime, this);


  timer_sync_ = nh_.createWallTimer(ros::WallDuration(1.00),&FcuBinder::timerSync,this);
  // | ----------------------- finish init ---------------------- |
  umsg_CRCInit();
  is_initialized_ = true;
  recvThread_ = std::thread([this] {this->Receiver();});
  
  ROS_INFO("[FcuBinder]: initialized");
}

//}

// | ------------------------- timers ------------------------- |


// | ------------------------- callbacks ------------------------- |
  void FcuBinder::callbackTime(const  rosgraph_msgs::Clock::ConstPtr msg){
    auto sim_time= msg->clock;
    mrs_lib::set_mutexed(mutex_sim_time_,sim_time,sim_time_);
  }


  void FcuBinder::callbackOdometry(const nav_msgs::Odometry::ConstPtr msg){

    //ROS_INFO("[FcuBinder]: ODOM CALLBACK called");
  }


  void FcuBinder::callbackIMU(const  sensor_msgs::Imu::ConstPtr msg){
    const int rate_limit = 20;
    static ros::Time last_published;
    if(! is_synced_){
      auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_,sim_time_);
      last_published = sim_time;
      return;
    }
    
    // fill the packet header
    umsg_MessageToTransfer out;
    auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_,sim_time_);
    
    if(last_published == sim_time){
      return;
    }
    else{
      last_published = sim_time;
    }
    
    std::scoped_lock(serial_mutex_);
    
    
    uint32_t timestamp = RosToFcu(sim_time);
    out.s.sync0 = 'M';
    out.s.sync1 = 'R';
    
    
    out.s.msg_class = UMSG_SENSORS;
    out.s.msg_type = SENSORS_IMU;
    out.s.sensors.imu.accel[0] = static_cast<float>(msg->linear_acceleration.x / GRAV_CONST);
    out.s.sensors.imu.accel[1] = static_cast<float>(msg->linear_acceleration.y / GRAV_CONST);
    out.s.sensors.imu.accel[2] = static_cast<float>(msg->linear_acceleration.z / GRAV_CONST);
    
    out.s.sensors.imu.gyro[0] = static_cast<float>(msg->angular_velocity.x);
    out.s.sensors.imu.gyro[1] = static_cast<float>(msg->angular_velocity.y);
    out.s.sensors.imu.gyro[2] = static_cast<float>(msg->angular_velocity.z);
    out.s.sensors.imu.timestamp = timestamp;

    uint32_t len = UMSG_HEADER_SIZE;
    len+=sizeof(umsg_sensors_imu_t) + 1;
    out.s.len = len;
    out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
    ser.sendCharArray(out.raw,out.s.len);
    if(rate_divider >= rate_limit)
    {
      geometry_msgs::Quaternion orient = msg->orientation;
      Eigen::Matrix3d Rd = mrs_lib::AttitudeConverter(orient);
      Eigen::Matrix3f R = Rd.cast<float>();
      
      out.s.msg_class = UMSG_SENSORS;
      out.s.msg_type = SENSORS_MAG;
      //ROS_INFO("[FCU BINDER] %f %f %f",R(0,0),R(1,0),R(2,0));
      out.s.sensors.mag.mag[0] = R(0,0);
      out.s.sensors.mag.mag[1] = R(1,0);
      out.s.sensors.mag.mag[2] = R(2,0);
      out.s.sensors.mag.timestamp = timestamp;

      len = UMSG_HEADER_SIZE;
      len+=sizeof(umsg_sensors_mag_t) + 1;
      out.s.len = len;
      out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
      ser.sendCharArray(out.raw,out.s.len);
    }

    out.s.msg_class = UMSG_SENSORS;
    out.s.msg_type = SENSORS_NOTIFYSENSORDATA;
    out.s.sensors.notifySensorData.imu = 1;
    out.s.sensors.notifySensorData.altimeter = 0;
    out.s.sensors.notifySensorData.baro = 0;
    out.s.sensors.notifySensorData.GPS = 0;
    if(rate_divider>=rate_limit){
      out.s.sensors.notifySensorData.magnetometer = 1;
    }
    else{
      out.s.sensors.notifySensorData.magnetometer = 0;
    }

    out.s.sensors.notifySensorData.timestamp = timestamp;

    len = UMSG_HEADER_SIZE;
    len+=sizeof(umsg_sensors_notifySensorData_t) + 1;
    out.s.len = len;
    out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
    ser.sendCharArray(out.raw,out.s.len);
    //ROS_INFO("[FcuBinder]: IMU Duration %d",diff_to_now.toNSec());
    ROS_INFO("[FcuBinder]: IMU CALLBACK called with timestamp %ld",timestamp);
    // toto send the message over the serial

    if(rate_divider>=rate_limit){
      rate_divider = 0;
    }
    else{
      rate_divider++;
    }
  }



  void FcuBinder::callbackRangeFinder(const  sensor_msgs::Range::ConstPtr msg){
   
   /*
    static ros::Time last_published;
    if(! is_synced_){
      auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_,sim_time_);
      last_published = sim_time;
      return;
    }
    // fill the packet header
    umsg_MessageToTransfer out;
    auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_,sim_time_);

    if(last_published == sim_time){
      return;
    }
    else{
      last_published = sim_time;
    }

    uint32_t timestamp = RosToFcu(sim_time);
    
    
    out.s.sync0 = 'M';
    out.s.sync1 = 'R';
    out.s.msg_class = UMSG_SENSORS;
    out.s.msg_type = SENSORS_ALTIMETER;
    
    out.s.sensors.altimeter.altitude = msg->range;
    out.s.sensors.altimeter.timestamp = timestamp;
    uint32_t len = UMSG_HEADER_SIZE;
    len+=sizeof(umsg_sensors_altimeter_t) + 1;
    out.s.len = len;
    out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
    //ser.write(out.raw,out.s.len);

    out.s.msg_class = UMSG_SENSORS;
    out.s.msg_type = SENSORS_NOTIFYSENSORDATA;
    out.s.sensors.notifySensorData.imu = 0;
    out.s.sensors.notifySensorData.altimeter = 1;
    out.s.sensors.notifySensorData.baro = 0;
    out.s.sensors.notifySensorData.GPS = 0;
    out.s.sensors.notifySensorData.magnetometer = 0;

    out.s.sensors.notifySensorData.timestamp = timestamp;

    len = UMSG_HEADER_SIZE;
    len+=sizeof(umsg_sensors_notifySensorData_t) + 1;
    out.s.len = len;
    out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
    //ser.write(out.raw,out.s.len);
    //ser.flushOutput();
    
    //ROS_INFO("[FcuBinder]: Range Duration %d",diff_to_now.toNSec());
    //ROS_INFO("[FcuBinder]: Range CALLBACK called with timestamp %d",timestamp);
  */
  }


  void FcuBinder::Receiver()
  {

    if (!is_initialized_)
    {
      return;
    }
    bool receptionComplete = false;
    ROS_INFO_ONCE("[FcuBinder]: ReceiverActive spinning");
    size_t readBytes = 0; // amount of read bytes
    size_t toRead = 0; // amount of read bytes
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
            else{
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
            else{
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
            else{
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
                msg_len +=recvdMsg.s.len - msg_len;
                receptionComplete = true;
                readBytes = 0;
            }
            else{
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
        //ROS_INFO("[FcuBinder]: received message of class %d and type %d", recvdMsg.s.msg_class, recvdMsg.s.msg_type);
        if (recvdMsg.s.msg_class == UMSG_CONTROL && recvdMsg.s.msg_type == CONTROL_DSHOTMESSAGE)
        {
          
          
          mrs_msgs::HwApiActuatorCmd cmd;
          umsg_control_DshotMessage_t DshotMessage = recvdMsg.s.control.DshotMessage;
          cmd.stamp = FcuToRos(DshotMessage.timestamp); 

          for (size_t i = 0; i < 4; i++)
          {
            cmd.motors.push_back(static_cast<float>(DshotMessage.channels[i])/2048.);
          }
          ph_actuator_cmd_.publish(cmd);

        }
        else if (recvdMsg.s.msg_class == UMSG_STATE && recvdMsg.s.msg_type == STATE_HEARTBEAT)
        {


          umsg_state_heartbeat_t beat = recvdMsg.s.state.heartbeat;

          calculateDelay(beat);
          if(! is_synced_){
              is_synced_ = true;
          }
        }

        // flush
        goto msg_flush;
      }

    if(toRead > 0)
    {
    readBytes = ser.readSerial(recvdMsg.raw + msg_len, toRead);
    toRead = 0;
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

    return;
  }

  void           FcuBinder::timerSync(const ros::WallTimerEvent& event){

    if (!is_initialized_)
    {
      return;
    }
    ROS_INFO_ONCE("[FcuBinder]: sync timer spinning");

    auto sequential   = mrs_lib::get_mutexed(mutex_sync_time,sequence_number);
    

    umsg_MessageToTransfer msg;

    msg.s.sync0 = 'M';
    msg.s.sync1 = 'R';
    msg.s.len = UMSG_HEADER_SIZE + sizeof(umsg_state_heartbeat_t) + 1;
    msg.s.state.heartbeat.seq_num = sequential;
    msg.s.state.heartbeat.timestamp_arrived = 0;
    msg.s.msg_class = UMSG_STATE;
    msg.s.msg_type = STATE_HEARTBEAT;
    msg.raw[msg.s.len-1] = umsg_calcCRC(msg.raw,msg.s.len-1);
    
    sequential +=1;
    
    auto curr_time   = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
    
    {
      std::scoped_lock(serial_mutex_);
      ser.sendCharArray(msg.raw,msg.s.len);
    }

    //auto curr_time   = ros::time::now();
    mrs_lib::set_mutexed(mutex_sync_time, std::tuple(curr_time,sequential), std::forward_as_tuple(sync_time_ROS_send,sequence_number));

    return;
  }





// | ------------------------- methods ------------------------ |

  void    FcuBinder::calculateDelay(umsg_state_heartbeat_t heartbeat){
          auto curr_time   = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
          //auto curr_time   = ros::Time::now();
          auto [start_time,sequential]   = mrs_lib::get_mutexed(mutex_sync_time, sync_time_ROS_send,sequence_number);

          ros::Duration diff;
          diff.fromNSec((curr_time - start_time).toNSec()/2);

          ros::Time syncTime_R = start_time + diff;
          uint32_t  syncTime_F = heartbeat.timestamp_arrived;

          if(heartbeat.seq_num == sequential - 1){
            mrs_lib::set_mutexed(mutex_sync_result,std::make_tuple(syncTime_R, syncTime_F),sync_result);

            double time_difference = static_cast<double>((curr_time - start_time).toNSec())/1e6;
            ROS_INFO("[SYNC] curr_time %ld start was %ld delay was %.3f miliseconds",curr_time.toNSec(),start_time.toNSec(),time_difference);
          }
          else{
            ROS_ERROR("[SYNC] NOT MATCHING SEQUENCE NUMBERS");
          }

  }

  uint32_t FcuBinder::RosToFcu(ros::Time &rosTime){

      auto [syncTime_R,syncTime_F]   = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
      int64_t diff = (rosTime.toNSec() - syncTime_R.toNSec())/1e6;
      int64_t new_stamp = diff + static_cast<int64_t>(syncTime_F);
      return static_cast<uint32_t>(new_stamp);
  }


  ros::Time FcuBinder::FcuToRos(uint32_t &FcuTime){
      auto [syncTime_R,syncTime_F]   = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
      int64_t diff = static_cast<int64_t>(FcuTime) - static_cast<int64_t>(syncTime_F)*1e6;

      ros::Duration diff_R;
      diff_R.fromNSec(diff);


      return syncTime_R + diff_R;
  }







}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_hitl_binders::FcuBinder, nodelet::Nodelet)


