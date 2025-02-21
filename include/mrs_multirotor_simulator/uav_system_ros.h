#ifndef UAV_SYSTEM_ROS_H
#define UAV_SYSTEM_ROS_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/transform_broadcaster.h>



#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_multirotor_simulator/uav_system/uav_system.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/MagneticField.h>

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

#include <random>
#include "iir_filter.h"

namespace mrs_multirotor_simulator
{

class UavSystemRos {

public:
  UavSystemRos(ros::NodeHandle& nh, const std::string name);

  void makeStep(const double dt,const ros::Time &sim_time);

  void crash(void);

  bool hasCrashed(void);

  void applyForce(const Eigen::Vector3d& force);

  Eigen::Vector3d getPose(void);

  MultirotorModel::ModelParams getParams();
  MultirotorModel::State       getState();

private:
  std::atomic<bool> is_initialized_ = false;
  std::string       _uav_name_;

  double randd(double from, double to);

  bool   _randomization_enabled_;
  double _randomization_bounds_x_;
  double _randomization_bounds_y_;
  double _randomization_bounds_z_;

  // | ------------------------ UavSystem ----------------------- |

  UavSystem::INPUT_MODE last_input_mode_;

  UavSystem  uav_system_;
  std::mutex mutex_uav_system_;

  ros::Time  time_last_input_;
  std::mutex mutex_time_last_input_;

  MultirotorModel::ModelParams model_params_;

  bool   _iterate_without_input_;
  double _input_timeout_;

  std::string _frame_world_;
  std::string _frame_fcu_;
  std::string _frame_rangefinder_;
  bool        _publish_rangefinder_tf_;
  bool        _publish_fcu_tf_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<sensor_msgs::Imu>   ph_imu_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_odom_;
  mrs_lib::PublisherHandler<sensor_msgs::Range> ph_rangefinder_;
  mrs_lib::PublisherHandler<sensor_msgs::MagneticField> ph_mag_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_altitude_;
  
  mrs_lib::PublisherHandler<sensor_msgs::Imu>   ph_imu_noise_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_odom_noise_;
  mrs_lib::PublisherHandler<sensor_msgs::Range> ph_rangefinder_noise_;
  mrs_lib::PublisherHandler<sensor_msgs::MagneticField> ph_mag_noise_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_altitude_noise_;

  void publishOdometry(const MultirotorModel::State& state, const ros::Time & sim_time);
  void publishIMU(const MultirotorModel::State& state, const ros::Time & sim_time);
  void publishRangefinder(const MultirotorModel::State& state, const ros::Time & sim_time);
  void publishMag(const MultirotorModel::State& state, const ros::Time & sim_time);
  void publishAltitude(const MultirotorModel::State& state, const ros::Time & sim_time);



  void timeoutInput(void);

  // | --------------------------- tf --------------------------- |

  std::shared_ptr<mrs_lib::TransformBroadcaster> tf_broadcaster_;

  // | ----------------------- subscribers ---------------------- |

  void callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg);
  void callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg);
  void callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg);
  void callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg);
  void callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg);
  void callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg);
  void callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg);
  void callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg);
  void callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg);
  void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg);

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>            sh_actuator_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>        sh_control_group_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>        sh_attitude_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>            sh_attitude_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> sh_acceleration_hdg_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>     sh_acceleration_hdg_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>     sh_velocity_hdg_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>         sh_velocity_hdg_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>            sh_position_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>              sh_tracker_cmd_;

  // | --------------------- service servers -------------------- |

  ros::ServiceServer service_server_set_mass_;

  ros::ServiceServer service_server_set_ground_z_;

  bool callbackSetMass(mrs_msgs::Float64Srv::Request& req, mrs_msgs::Float64Srv::Response& res);

  bool callbackSetGroundZ(mrs_msgs::Float64Srv::Request& req, mrs_msgs::Float64Srv::Response& res);

  // | ------------------------ routines ------------------------ |

  void calculateInertia(MultirotorModel::ModelParams& params);

  // | ------------------------- noise parameters ------------------------- |
    std::mt19937 gen;
    std::normal_distribution<double> accel_gen_;
    std::normal_distribution<double> gyro_gen_;
    std::normal_distribution<double> mag_gen_;
    std::normal_distribution<double> altitude_gen_;
    std::normal_distribution<double> position_gen_;
    std::normal_distribution<double> range_gen_;

    std::vector<mrs_lib::IirFilter> accel_noiseShapers_;
    std::vector<mrs_lib::IirFilter> gyro_noiseShapers_;
    std::vector<mrs_lib::IirFilter> mag_noiseShapers_;
    mrs_lib::IirFilter altitude_noiseShaper_;
    mrs_lib::IirFilter range_noiseShaper_;
    std::vector<mrs_lib::IirFilter> position_noiseShapers_;

    ros::Duration imu_delay_;
    ros::Duration mag_delay_;
    ros::Duration altitude_delay_;
    ros::Duration position_delay_;
    ros::Duration range_delay_;
    
    ros::Time imu_last_stamp_;
    ros::Time mag_last_stamp_;
    ros::Time altitude_last_stamp_;
    ros::Time position_last_stamp_;
    ros::Time range_last_stamp_;



};

}  // namespace mrs_multirotor_simulator

#endif  // UAV_SYSTEM_ROS_H
