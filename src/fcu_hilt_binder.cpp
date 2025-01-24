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

#include <serial/serial.h>

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
  serial::Serial ser;
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_;
  std::string uav_name;
  umsg_MessageToTransfer recvdMsg;
  uint32_t msg_len = 0;
  ReceiverState state = WAITING_FOR_SYNC0;
  // | ------------------------- params ------------------------- |

  ros::Time  sim_time_;
  std::mutex mutex_sim_time_;
  int port_fd;
  uint32_t last_sync_time;

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
  
  void publishActuatorCmd(const mrs_msgs::HwApiActuatorCmd &msg);
  
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

  ros::Time last_published_time_;
  // | -------------------------- Timers -------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  // | ------------------------- methods ------------------------ |


  // | --------------- dynamic reconfigure server --------------- |

};

//}

/* onInit() //{ */

void FcuBinder::onInit() {

  is_initialized_ = false;

  try {
        // Configure the serial port
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(1152000);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("[FcuBinder] Unable to open port ");
        return;
    }

    if (ser.isOpen()) {
        ROS_INFO("[FcuBinder] Serial port initialized");
    } else {
        return;
    }


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



  std::string uav_name;

  param_loader.loadParam("uav_name", uav_name);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[FcuBinder]: could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |
  ph_actuator_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>(nh_,uav_name + "/actuators_cmd",1,false);
  //ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>(nh_,uav_name + "/control_group_cmd",1,false);

  // | ----------------------- subscribers ----------------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = uav_name;
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();


  sh_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts,"imu",&FcuBinder::callbackIMU,this);
  sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts,"odom",&FcuBinder::callbackOdometry,this);
  sh_rangefinder_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts,"rangefinder",&FcuBinder::callbackRangeFinder,this);
  sh_clock_ = mrs_lib::SubscribeHandler<rosgraph_msgs::Clock>(shopts, "clock_in", &FcuBinder::callbackTime, this);


  timer_main_ = nh_.createWallTimer(ros::WallDuration(0.002), &FcuBinder::timerMain, this);
  // | ----------------------- finish init ---------------------- |
  umsg_CRCInit();
  is_initialized_ = true;
  
  ROS_INFO("[FcuBinder]: initialized");
}

//}

// | ------------------------- timers ------------------------- |



// | ------------------------- callbacks ------------------------- |
  void FcuBinder::callbackTime(const  rosgraph_msgs::Clock::ConstPtr msg){
    sim_time_ = msg->clock;
  }


  void FcuBinder::callbackOdometry(const nav_msgs::Odometry::ConstPtr msg){

    //ROS_INFO("[FcuBinder]: ODOM CALLBACK called");
  }


  void FcuBinder::callbackIMU(const  sensor_msgs::Imu::ConstPtr msg){
    // fill the packet header
    umsg_MessageToTransfer out;
    ros::Time t = msg->header.stamp;
    uint32_t timestamp = static_cast<uint32_t>(t.toNSec()/1e6);
    
    out.s.sync0 = 'M';
    out.s.sync1 = 'R';
    
    out.s.msg_class = UMSG_SENSORS;
    out.s.msg_type = SENSORS_NOTIFYSENSORDATA;
    out.s.sensors.notifySensorData.imu = 1;
    out.s.sensors.notifySensorData.altimeter = 0;
    out.s.sensors.notifySensorData.baro = 0;
    out.s.sensors.notifySensorData.GPS = 0;
    out.s.sensors.notifySensorData.magnetometer = 0;

    out.s.sensors.notifySensorData.timestamp = timestamp;

    uint32_t len = UMSG_HEADER_SIZE;
    len+=sizeof(umsg_sensors_notifySensorData_t) + 1;
    out.s.len = len;
    out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
    ser.write(out.raw,out.s.len);
    
    out.s.msg_class = UMSG_SENSORS;
    out.s.msg_type = SENSORS_IMU;
    out.s.sensors.imu.accel[0] = static_cast<float>(msg->linear_acceleration.x / GRAV_CONST);
    out.s.sensors.imu.accel[1] = static_cast<float>(msg->linear_acceleration.y / GRAV_CONST);
    out.s.sensors.imu.accel[2] = static_cast<float>(msg->linear_acceleration.z / GRAV_CONST);
    out.s.sensors.imu.gyro[0] = static_cast<float>(msg->angular_velocity.x);
    out.s.sensors.imu.gyro[1] = static_cast<float>(msg->angular_velocity.y);
    out.s.sensors.imu.gyro[2] = static_cast<float>(msg->angular_velocity.z);

    out.s.sensors.imu.timestamp = timestamp;

    len = UMSG_HEADER_SIZE;
    len+=sizeof(umsg_sensors_imu_t) + 1;
    out.s.len = len;
    out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
    ser.write(out.raw,out.s.len);
    
    
    //ROS_INFO("[FcuBinder]: IMU CALLBACK called");
    // toto send the message over the serial
  }
  void FcuBinder::callbackRangeFinder(const  sensor_msgs::Range::ConstPtr msg){
    // fill the packet header
    umsg_MessageToTransfer out;
    out.s.sync0 = 'M';
    out.s.sync1 = 'R';
    out.s.msg_class = UMSG_SENSORS;
    out.s.msg_type = SENSORS_ALTIMETER;
    
    out.s.sensors.altimeter.altitude = msg->range;
    ros::Time t = msg->header.stamp;
    uint32_t timestamp = static_cast<uint32_t>(t.toNSec()/1e6);
    out.s.sensors.altimeter.timestamp = timestamp;

    uint32_t len = UMSG_HEADER_SIZE;
    len+=sizeof(umsg_sensors_altimeter_t) + 1;
    out.s.len = len;
    out.raw[len-1] = umsg_calcCRC(out.raw,len-1);
    ser.write(out.raw,out.s.len);
    //ROS_INFO("[FcuBinder]: Range CALLBACK called");
  }


  void           FcuBinder::timerMain(const ros::WallTimerEvent& event){

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[FcuBinder]: main timer spinning");
    state = WAITING_FOR_SYNC0;
    bool receptionComplete = false;
    uint32_t amount_to_flush = 0; // how much of the buffer needs to be destroyed because of invalid data
    
    while(!receptionComplete){

        size_t val = ser.available();
        // NOTE: potential issue with buffer overflow here
        if( val > 0){
          uint32_t to_read = std::min(sizeof(umsg_MessageToTransfer) - msg_len, val);
          ROS_INFO("[FcuBinder]: reading bytes: %d", to_read);
          ser.read(recvdMsg.raw + msg_len,to_read);
          msg_len+= val;      
        }
        switch (state)
        {
        case WAITING_FOR_SYNC0:
            {
                if(msg_len >= 1){
                    if(recvdMsg.s.sync0 != 'M'){
                        amount_to_flush = 1;
                        goto msg_err;
                    }
                    state = WAITING_FOR_SYNC1;
                }
                break;
            }
        case WAITING_FOR_SYNC1:
            {
                if(msg_len >= 2){
                    if(recvdMsg.s.sync1 != 'R'){
                        amount_to_flush = 2;
                        goto msg_err;
                    }
                    state = WAITING_FOR_HEADER;
                }
                break;
            }
        case WAITING_FOR_HEADER:
            {
                if(msg_len >= UMSG_HEADER_SIZE)
                {
                    if(recvdMsg.s.len > sizeof(umsg_MessageToTransfer)){
                        amount_to_flush = UMSG_HEADER_SIZE;
                        goto msg_err;
                    }
                    state = WAITING_FOR_PAYLOAD;                    
                }
            }
        //fall through
        case WAITING_FOR_PAYLOAD:
            {
                if(msg_len >= recvdMsg.s.len){
                    if(umsg_calcCRC(recvdMsg.raw,recvdMsg.s.len-1) != recvdMsg.raw[recvdMsg.s.len-1]){
                        amount_to_flush = msg_len;
                        goto msg_err;
                    }
                    receptionComplete = true;
                }
            }
            break;

        default:
            goto msg_err;
            amount_to_flush = 0;
            break;
        }
    }

    if(receptionComplete){
      ROS_INFO("[FcuBinder]: received message of class %d and type %d", recvdMsg.s.msg_class, recvdMsg.s.msg_type);
      if(recvdMsg.s.msg_class == UMSG_CONTROL && recvdMsg.s.msg_type == CONTROL_DSHOTMESSAGE){

        ROS_INFO("[FcuBinder]: Actuator command received");
        mrs_msgs::HwApiActuatorCmd cmd;
        cmd.stamp = sim_time_; // TODO: add correct timestamp
        for (size_t i = 0; i < 4; i++)
        {
          cmd.motors.push_back(static_cast<float>(recvdMsg.s.control.DshotMessage.channels[i])/2048.);
        }

      }

      amount_to_flush = recvdMsg.s.len;
      for (size_t i = 0; i < msg_len - amount_to_flush; i++)
      {
        uint32_t indice = std::max(sizeof(umsg_MessageToTransfer), i + amount_to_flush);
        recvdMsg.raw[i] = recvdMsg.raw[indice];
      }
      msg_len += -amount_to_flush;
      return;
    }



    msg_err:
    
    // NOTE: another buffer overflow issue here
    ROS_ERROR("message corrupter, flushing: %d",amount_to_flush);
    for (size_t i = 0; i < msg_len - amount_to_flush; i++)
    {
      uint32_t indice = std::max(sizeof(umsg_MessageToTransfer), i + amount_to_flush);
      recvdMsg.raw[i] = recvdMsg.raw[indice];
    }
    msg_len += -amount_to_flush;

    return;  
  }







}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_hitl_binders::FcuBinder, nodelet::Nodelet)


