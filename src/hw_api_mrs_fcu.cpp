/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/api.h>

#include <std_srvs/Trigger.h>

#include <mrs_modules_msgs/Bestpos.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/gps_conversions.h>

#include <std_msgs/Float64.h>

#include <geometry_msgs/QuaternionStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/GPSRAW.h>

#include "SerialApi.h"

#include <umsg.h>
#include <umsg_classes.h>

//}

/* defines //{ */

#define GRAV_CONST 9.81
#define PWM_MIDDLE 1500
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN

//}

namespace mrs_uav_fcu_api
{
    class hitl_binder
    {
    private:
        std::shared_ptr<SerialApi> ser_;
        double startX, startY;
        std::string UTM_zone;
        // | ----------------------- subscribers ----------------------- |
        mrs_lib::SubscribeHandler<sensor_msgs::Imu> sh_imu_;
        mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odom_;
        mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_rangefinder_;
        mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_altitude_;
        mrs_lib::SubscribeHandler<sensor_msgs::MagneticField> sh_mag_;

        void callbackOdometry(const nav_msgs::Odometry::ConstPtr msg);
        void callbackIMU(const sensor_msgs::Imu::ConstPtr msg);
        void callbackRangeFinder(const sensor_msgs::Range::ConstPtr msg);
        void callbackAltitude(const nav_msgs::Odometry::ConstPtr msg);
        void callbackMag(const sensor_msgs::MagneticField::ConstPtr msg);

        void publishImu(const sensor_msgs::Imu::ConstPtr msg, ros::Time &sim_time);
        void publishMag(const sensor_msgs::MagneticField::ConstPtr msg, ros::Time &sim_time);
        void publishAltitude(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time);
        void publishGps(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time);

        // | ----------------------- publishers ----------------------- |
        mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd> ph_actuator_cmd_;

    public:
        void Init(const ros::NodeHandle &parent_nh, std::shared_ptr<SerialApi> ser);
        bool ParseMessage(umsg_MessageToTransfer &msg);
    };

    void hitl_binder::Init(const ros::NodeHandle &parent_nh, std::shared_ptr<SerialApi> ser)
    {
        ser_ = ser;
        double startLat, startLon;

        ros::NodeHandle nh_(parent_nh);

        mrs_lib::ParamLoader param_loader(nh_, "hitl_binder");
        param_loader.loadParam("start_latitude", startLat);
        param_loader.loadParam("start_longditude", startLon);

        mrs_lib::LLtoUTM(startLat, startLon, startY, startX, UTM_zone);

        mrs_lib::SubscribeHandlerOptions shopts;
        shopts.nh = nh_;
        shopts.node_name = "hitl_binder";
        shopts.no_message_timeout = mrs_lib::no_timeout;
        shopts.threadsafe = true;
        shopts.autostart = true;
        shopts.queue_size = 1;
        shopts.transport_hints = ros::TransportHints().tcpNoDelay();
        sh_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "hitl/imu", &hitl_binder::callbackIMU, this);
        sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "hitl/odom", &hitl_binder::callbackOdometry, this);
        // sh_rangefinder_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "hitl/rangefinder", &hitl_binder::callbackRangeFinder, this);
        sh_altitude_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "hitl/altitude", &hitl_binder::callbackAltitude, this);
        sh_mag_ = mrs_lib::SubscribeHandler<sensor_msgs::MagneticField>(shopts, "hitl/magnetometer", &hitl_binder::callbackMag, this);

        ROS_INFO("SELECTED UTM ZONE IS : %s", UTM_zone.c_str());

        ph_actuator_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>(nh_, "hitl/actuators_cmd", 1, false);
    };

    // PublishImu//{
    void hitl_binder::publishImu(const sensor_msgs::Imu::ConstPtr msg, ros::Time &sim_time)
    { /*//{*/
        static double index = 0;
        umsg_MessageToTransfer out;
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
        out.s.sensors.imu.timestamp = ser_->RosToFcu(sim_time);
        out.s.sensors.imu.temperature = index;
        index += 1;
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_imu_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);
        ser_->sendPacket(out);
    } /*//}*/ /*//}*/

    void hitl_binder::publishMag(const sensor_msgs::MagneticField::ConstPtr msg, ros::Time &sim_time)
    {

        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';

        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_MAG;
        // ROS_INFO("[FCU BINDER] %f %f %f",R(0,0),R(1,0),R(2,0));
        out.s.sensors.mag.mag[0] = static_cast<float>(msg->magnetic_field.x);
        out.s.sensors.mag.mag[1] = static_cast<float>(msg->magnetic_field.y);
        out.s.sensors.mag.mag[2] = static_cast<float>(msg->magnetic_field.z);
        out.s.sensors.mag.timestamp = ser_->RosToFcu(sim_time);

        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_mag_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);
        ser_->sendPacket(out);
    }

    void hitl_binder::publishAltitude(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time)
    {
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_ALTIMETER;

        out.s.sensors.altimeter.altitude = static_cast<float>(msg->pose.pose.position.z);
        out.s.sensors.altimeter.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_altimeter_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);
        ser_->sendPacket(out);
    }

    void hitl_binder::publishGps(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time)
    {
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_GPS;

        out.s.sensors.gps.timestamp = ser_->RosToFcu(sim_time);
        out.s.sensors.gps.fixType = FIX_3D;
        out.s.sensors.gps.hELPS = 0;
        out.s.sensors.gps.hMSL = 0;
        out.s.sensors.gps.reserved = 0;
        out.s.sensors.gps.numSV = 20;

        double UTMNorth, UTMEast;
        UTMEast = startX + msg->pose.pose.position.x;
        UTMNorth = startY + msg->pose.pose.position.y;

        double lat, lon;
        mrs_lib::UTMtoLL(UTMNorth, UTMEast, UTM_zone, lat, lon);

        out.s.sensors.gps.lat = lat;
        out.s.sensors.gps.lon = lon;

        out.s.sensors.gps.CRCValid = 1;
        out.s.sensors.gps.DataValid = 1;
        out.s.sensors.gps.gnssFixOk = 1;

        out.s.sensors.gps.vel[0] = 0;
        out.s.sensors.gps.vel[1] = 0;
        out.s.sensors.gps.vel[2] = 0;

        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_gps_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);

        ser_->sendPacket(out);
    }

    // | ------------------------- callbacks ------------------------- |

    void hitl_binder::callbackOdometry(const nav_msgs::Odometry::ConstPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }
        // fill the packet header
        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        ros::Time sim_time = msg->header.stamp;
        publishGps(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.GPS = 1;
        ROS_INFO_ONCE("[HITLBinder]: GPS CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser_->sendPacket(notifyMsg);
    }

    void hitl_binder::callbackIMU(const sensor_msgs::Imu::ConstPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }
        // fill the packet header
        ros::Time sim_time = msg->header.stamp;

        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        publishImu(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.imu = 1;
        ROS_INFO_ONCE("[HITLBinder]: IMU CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser_->sendPacket(notifyMsg);
        //  ROS_INFO("[FcuBinder]: IMU Duration %d",diff_to_now.toNSec());
        //   toto send the message over the serial
    }

    void hitl_binder::callbackRangeFinder(const sensor_msgs::Range::ConstPtr msg)
    {
        ROS_WARN_ONCE("rangefinder callback not yet implemented");
    }

    void hitl_binder::callbackAltitude(const nav_msgs::Odometry::ConstPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }
        // fill the packet header
        ros::Time sim_time = msg->header.stamp;

        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        publishAltitude(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.altimeter = 1;
        ROS_INFO_ONCE("[HITLBinder]: Altitude CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser_->sendPacket(notifyMsg);
        // ROS_INFO("[FcuBinder]: IMU Duration %d",diff_to_now.toNSec());
        //  toto send the message over the serial
    }
    void hitl_binder::callbackMag(const sensor_msgs::MagneticField::ConstPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }
        // fill the packet header
        ros::Time sim_time = msg->header.stamp;

        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        publishMag(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.magnetometer = 1;
        ROS_INFO_ONCE("[FcuBinder]: mag CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser_->sendPacket(notifyMsg);
        // ROS_INFO("[FcuBinder]: IMU Duration %d",diff_to_now.toNSec());
        //  toto send the message over the serial
    }

    bool hitl_binder::ParseMessage(umsg_MessageToTransfer &msg)
    {
        uint8_t msg_class = msg.s.msg_class;
        uint8_t msg_type = msg.s.msg_type;

        bool parsed = true;

        switch (msg_class)
        {
        case UMSG_CONTROL:
        {
            switch (msg_type)
            {
                ROS_INFO("received DSHOT MSG");
            case CONTROL_DSHOTMESSAGE:
            {
                mrs_msgs::HwApiActuatorCmd cmd;
                umsg_control_DshotMessage_t DshotMessage = msg.s.control.DshotMessage;
                cmd.stamp = ser_->FcuToRos(DshotMessage.timestamp);

                for (size_t i = 0; i < 4; i++)
                {
                    cmd.motors.push_back(static_cast<float>(DshotMessage.channels[i]) / 2048.);
                }
                ph_actuator_cmd_.publish(cmd);
            }
            break;

            default:
                parsed = false;
                break;
            }
        }
        break;

        default:
            parsed = false;
            break;
        }
        return parsed;
    }

    /* class MrsUavFcuApi //{ */

    class MrsUavFcuApi : public mrs_uav_hw_api::MrsUavHwApi
    {

    public:
        ~MrsUavFcuApi() {};

        void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

        // | --------------------- status methods --------------------- |

        mrs_msgs::HwApiStatus getStatus();
        mrs_msgs::HwApiCapabilities getCapabilities();

        // | --------------------- topic callbacks -------------------- |

        bool callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg);
        bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg);
        bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg);
        bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg);
        bool callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg);
        bool callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg);
        bool callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg);
        bool callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg);
        bool callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg);
        void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg);
        // | -------------------- service callbacks ------------------- |

        std::tuple<bool, std::string> callbackArming(const bool &request);
        std::tuple<bool, std::string> callbackOffboard(void);

    private:
        bool is_initialized_ = false;

        std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

        // | ----------------------- parameters ----------------------- |

        mrs_msgs::HwApiCapabilities _capabilities_;

        std::string _uav_name_;
        std::string _body_frame_name_;
        std::string _world_frame_name_;

        double _comms_timeout_;

        bool _simulation_;

        std::shared_ptr<SerialApi> ser_;
        hitl_binder hitl_binder_;
        // output methods for rtk
        void publishGroundTruth(const nav_msgs::Odometry::ConstPtr msg);

        void publishRTK(const mrs_modules_msgs::Bestpos::ConstPtr msg);

        double RCChannelToRange(const double &rc_value);

        // normal output methods

        void publishState(const mavros_msgs::State::ConstPtr msg);

        void publishOdometryLocal(const nav_msgs::Odometry::ConstPtr msg);
        void publishNavsatFix(const sensor_msgs::NavSatFix::ConstPtr msg);
        void publishDistanceSensor(const sensor_msgs::Range::ConstPtr msg); // not yet implemented
        void publishImu(const umsg_sensors_imu_t &msg);
        void publishMagnetometer(const umsg_sensors_mag_t &msg);
        void publishMagneticField(const umsg_sensors_mag_t &msg);
        void publishRC(const umsg_control_sBusPacket_t &msg);
        void publishAltitude(const mavros_msgs::Altitude::ConstPtr msg);
        void publishGpsStatusRaw(const umsg_sensors_gps_t &msg);
        void publishBattery(const sensor_msgs::BatteryState::ConstPtr msg); // not yet implemented

        std::thread parser_thread_;
        void messageParser();

        bool ParseMessage(umsg_MessageToTransfer &msg);
        // | ------------------------ variables ----------------------- |

        std::string uav_name;
    };

    //}

    // --------------------------------------------------------------
    // |                   controller's interface                   |
    // --------------------------------------------------------------

    /* initialize() //{ */

    void MrsUavFcuApi::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers)
    {

        ros::NodeHandle nh_(parent_nh);

        common_handlers_ = common_handlers;

        _uav_name_ = common_handlers->getUavName();
        _body_frame_name_ = common_handlers->getBodyFrameName();
        _world_frame_name_ = common_handlers->getWorldFrameName();

        _capabilities_.api_name = "FcuApi";

        // | ------------------- loading parameters ------------------- |

        mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

        // ask what this does
        param_loader.loadParam("comms_timeout", _comms_timeout_);

        // ask what this does
        param_loader.loadParam("simulation", _simulation_);

        param_loader.loadParam("inputs/control_group", (bool &)_capabilities_.accepts_control_group_cmd);
        param_loader.loadParam("inputs/attitude_rate", (bool &)_capabilities_.accepts_attitude_rate_cmd);
        param_loader.loadParam("inputs/attitude", (bool &)_capabilities_.accepts_attitude_cmd);

        param_loader.loadParam("outputs/distance_sensor", (bool &)_capabilities_.produces_distance_sensor);
        param_loader.loadParam("outputs/gnss", (bool &)_capabilities_.produces_gnss);
        param_loader.loadParam("outputs/gnss_status", (bool &)_capabilities_.produces_gnss_status);
        param_loader.loadParam("outputs/rtk", (bool &)_capabilities_.produces_rtk);
        param_loader.loadParam("outputs/ground_truth", (bool &)_capabilities_.produces_ground_truth);
        param_loader.loadParam("outputs/imu", (bool &)_capabilities_.produces_imu);
        param_loader.loadParam("outputs/altitude", (bool &)_capabilities_.produces_altitude);
        param_loader.loadParam("outputs/magnetometer_heading", (bool &)_capabilities_.produces_magnetometer_heading);
        param_loader.loadParam("outputs/magnetic_field", (bool &)_capabilities_.produces_magnetic_field);
        param_loader.loadParam("outputs/rc_channels", (bool &)_capabilities_.produces_rc_channels);
        param_loader.loadParam("outputs/battery_state", (bool &)_capabilities_.produces_battery_state);
        param_loader.loadParam("outputs/position", (bool &)_capabilities_.produces_position);
        param_loader.loadParam("outputs/orientation", (bool &)_capabilities_.produces_orientation);
        param_loader.loadParam("outputs/velocity", (bool &)_capabilities_.produces_velocity);
        param_loader.loadParam("outputs/angular_velocity", (bool &)_capabilities_.produces_angular_velocity);
        param_loader.loadParam("outputs/odometry", (bool &)_capabilities_.produces_odometry);

        if (!param_loader.loadedSuccessfully())
        {
            ROS_ERROR("[MrsUavFcuApi]: Could not load all parameters!");
            ros::shutdown();
        }

        // | ----------------------- subscribers ---------------------- |

        mrs_lib::SubscribeHandlerOptions shopts;
        shopts.nh = nh_;
        shopts.node_name = "MrsHwFcuApi";
        shopts.no_message_timeout = mrs_lib::no_timeout;
        shopts.threadsafe = true;
        shopts.autostart = true;
        shopts.queue_size = 10;
        shopts.transport_hints = ros::TransportHints().tcpNoDelay();

        if (_simulation_)
        {
            // sh_ground_truth_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "ground_truth_in", &MrsUavFcuApi::callbackGroundTruth, this);
        }

        if (!_simulation_)
        {
            // sh_rtk_ = mrs_lib::SubscribeHandler<mrs_modules_msgs::Bestpos>(shopts, "rtk_in", &MrsUavFcuApi::callbackRTK, this);
        }

        // | ----------------------- publishers ----------------------- |

        // | ----------------------- finish init ---------------------- |

        std::string serial_port;
        param_loader.loadParam("serial_port", serial_port);
        int baud_rate;

        param_loader.loadParam("baud_rate", baud_rate);

        ser_ = std::make_shared<SerialApi>(serial_port, baud_rate);

        ser_->startReceiver();
        ser_->startSyncTimer(nh_);
        if (_simulation_)
        {
            hitl_binder_.Init(parent_nh, ser_);
        }
        ROS_INFO("[MrsUavFcuApi]: initialized");
        parser_thread_ = std::thread([this]
                                     { this->messageParser(); });
        is_initialized_ = true;
    }

    //}

    /* getStatus() //{ */

    mrs_msgs::HwApiStatus MrsUavFcuApi::getStatus()
    {

        mrs_msgs::HwApiStatus status;

        status.stamp = ros::Time::now();

        return status;
    }

    //}

    /* getCapabilities() //{ */

    mrs_msgs::HwApiCapabilities MrsUavFcuApi::getCapabilities()
    {

        _capabilities_.stamp = ros::Time::now();

        return _capabilities_;
    }

    //}

    /* callbackAttitudeRateCmd() //{ */

    bool MrsUavFcuApi::callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg)
    {

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting attitude rate cmd");

        if (!_capabilities_.accepts_attitude_rate_cmd)
        {
            ROS_ERROR_THROTTLE(1.0, "[MrsUavFcuApi]: attitude rate input is not enabled in the config file");
            return false;
        }

        umsg_MessageToTransfer out;

        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_OFFBOARD;
        out.s.msg_type = OFFBOARD_RATECMD;
        out.s.offboard.RateCmd.roll_rate = msg->body_rate.x;
        out.s.offboard.RateCmd.pitch_rate = msg->body_rate.y;
        out.s.offboard.RateCmd.yaw_rate = msg->body_rate.z;
        out.s.offboard.RateCmd.throttle = msg->throttle;
        out.s.offboard.RateCmd.timestamp = ser_->RosToFcu(msg->stamp);
        out.s.len = UMSG_HEADER_SIZE + sizeof(umsg_offboard_RateCmd_t) + 1;
        out.raw[out.s.len - 1] = umsg_calcCRC(out.raw, out.s.len - 1);
        ser_->sendPacket(out);

        return true;
    }

    //}

    /* callbackAttitudeCmd() //{ */

    bool MrsUavFcuApi::callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg)
    {

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting attitude cmd");

        if (!_capabilities_.accepts_attitude_cmd)
        {
            ROS_ERROR_THROTTLE(1.0, "[MrsUavFcuApi]: attitude input is not enabled in the config file");
            return false;
        }

        return true;
    }

    //}

    /* callbackArming() //{ */

    std::tuple<bool, std::string> MrsUavFcuApi::callbackArming([[maybe_unused]] const bool &request)
    {

        std::stringstream ss;

        mavros_msgs::CommandLong srv_out;

        // when REALWORLD AND ARM:=TRUE
        if (!_simulation_ && request)
        {

            ss << "can not arm by service when not in simulation! You should arm the drone by the RC controller only!";
            ROS_ERROR_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());

            return {false, "ss.str()"};
        }

        srv_out.request.broadcast = false;
        srv_out.request.command = 400; // the code for arming
        srv_out.request.confirmation = true;

        srv_out.request.param1 = request ? 1 : 0;     // arm or disarm?
        srv_out.request.param2 = request ? 0 : 21196; // 21196 allows to disarm even in mid-flight
        srv_out.request.param3 = 0;
        srv_out.request.param4 = 0;
        srv_out.request.param5 = 0;
        srv_out.request.param6 = 0;
        srv_out.request.param7 = 0;

        ROS_INFO("[Px4Api]: calling for %s", request ? "arming" : "disarming");

        return {false, "ss.str()"};
    }

    //}

    /* callbackOffboard() //{ */

    std::tuple<bool, std::string> MrsUavFcuApi::callbackOffboard(void)
    {

        mavros_msgs::SetMode srv;

        srv.request.base_mode = 0;
        srv.request.custom_mode = "OFFBOARD";

        std::stringstream ss;

        if (1)
        {

            ss << "Service call for offboard failed!";

            ROS_ERROR_THROTTLE(1.0, "[Px4Api]: %s", ss.str().c_str());
            return {false, ss.str()};
        }
        else
        {

            if (srv.response.mode_sent != 1)
            {

                ss << "service call for offboard failed, returned " << srv.response.mode_sent;

                ROS_WARN_THROTTLE(1.0, "[Px4Api]: %s", ss.str().c_str());

                return {false, ss.str()};
            }
            else
            {

                ss << "switched to offboard mode";

                return {true, ss.str()};
            }
        }
    }

    //}

    // | ------------------- additional methods ------------------- |

    /* timeoutMavrosState() //{ */
    /*
    void MrsUavFcuApi::timeoutMavrosState([[maybe_unused]] const std::string &topic, const ros::Time &last_msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        if (!sh_mavros_state_.hasMsg())
        {
            return;
        }

        ros::Duration time = ros::Time::now() - last_msg;

        if (time.toSec() > _comms_timeout_)
        {

            {
                std::scoped_lock lock(mutex_status_);

                connected_ = false;
                offboard_ = false;
                armed_ = false;
                mode_ = "";
            }

            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: Have not received Mavros state for more than '%.3f s'", time.toSec());
        }
        else
        {

            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: Not recieving Mavros state message for '%.3f s'! Setup the PixHawk SD card!!", time.toSec());
            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: This could be also caused by the not being PixHawk booted properly due to, e.g., antispark connector jerkyness.");
            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: The Mavros state should be supplied at 100 Hz to provided fast refresh rate on the state of the OFFBOARD mode.");
            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: If missing, the UAV could be disarmed by safety routines while not knowing it has switched to the MANUAL mode.");
        }
    }
    */
    //}

    /* RCChannelToRange() //{ */

    double MrsUavFcuApi::RCChannelToRange(const double &rc_value)
    {

        double tmp_0_to_1 = (rc_value - double(PWM_MIN)) / (double(PWM_RANGE));

        if (tmp_0_to_1 > 1.0)
        {
            tmp_0_to_1 = 1.0;
        }
        else if (tmp_0_to_1 < 0.0)
        {
            tmp_0_to_1 = 0.0;
        }

        return tmp_0_to_1;
    }

    //}

    // | ------------------------ callbacks ----------------------- |

    /* //{ callbackMavrosState() */

    /*
    void MrsUavFcuApi::publishMavrosState(const mavros_msgs::State::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting Mavros state");

        {
            std::scoped_lock lock(mutex_status_);

            offboard_ = msg->mode == "OFFBOARD";
            armed_ = msg->armed;
            connected_ = true;
            mode_ = msg->mode;
        }

        // | ----------------- publish the diagnostics ---------------- |

        mrs_msgs::HwApiStatus status;

        {
            std::scoped_lock lock(mutex_status_);

            status.stamp = ros::Time::now();
            status.armed = armed_;
            status.offboard = offboard_;
            status.connected = connected_;
            status.mode = mode_;
        }

        common_handlers_->publishers.publishStatus(status);
    }

    */
    //}

    /* callbackOdometryLocal() //{ */

    void MrsUavFcuApi::publishOdometryLocal(const nav_msgs::Odometry::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting Mavros's local odometry");

        auto odom = msg;

        // | -------------------- publish position -------------------- |

        if (_capabilities_.produces_position)
        {

            geometry_msgs::PointStamped position;

            position.header.stamp = odom->header.stamp;
            position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
            position.point = odom->pose.pose.position;

            common_handlers_->publishers.publishPosition(position);
        }

        // | ------------------- publish orientation ------------------ |

        if (_capabilities_.produces_orientation)
        {

            geometry_msgs::QuaternionStamped orientation;

            orientation.header.stamp = odom->header.stamp;
            orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
            orientation.quaternion = odom->pose.pose.orientation;

            common_handlers_->publishers.publishOrientation(orientation);
        }

        // | -------------------- publish velocity -------------------- |

        if (_capabilities_.produces_velocity)
        {

            geometry_msgs::Vector3Stamped velocity;

            velocity.header.stamp = odom->header.stamp;
            velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
            velocity.vector = odom->twist.twist.linear;

            common_handlers_->publishers.publishVelocity(velocity);
        }

        // | ---------------- publish angular velocity ---------------- |

        if (_capabilities_.produces_angular_velocity)
        {

            geometry_msgs::Vector3Stamped angular_velocity;

            angular_velocity.header.stamp = odom->header.stamp;
            angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
            angular_velocity.vector = odom->twist.twist.angular;

            common_handlers_->publishers.publishAngularVelocity(angular_velocity);
        }

        // | -------------------- publish odometry -------------------- |

        if (_capabilities_.produces_odometry)
        {
            common_handlers_->publishers.publishOdometry(*odom);
        }
    }

    //}

    /* callbackNavsatFix() //{ */

    void MrsUavFcuApi::publishNavsatFix(const sensor_msgs::NavSatFix::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting NavSat fix");

        if (_capabilities_.produces_gnss)
        {

            common_handlers_->publishers.publishGNSS(*msg);
        }
    }

    //}

    /* callbackDistanceSensor() //{ */

    void MrsUavFcuApi::publishDistanceSensor(const sensor_msgs::Range::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting distnace sensor");

        if (_capabilities_.produces_distance_sensor)
        {

            common_handlers_->publishers.publishDistanceSensor(*msg);
        }
    }

    //}

    /* callbackImu() //{ */

    void MrsUavFcuApi::publishImu(const umsg_sensors_imu_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting IMU");

        if (_capabilities_.produces_imu)
        {

            /*
            sensor_msgs::Imu new_imu_msg = *msg;
            new_imu_msg.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

            common_handlers_->publishers.publishIMU(new_imu_msg);
            */
        }
    }

    //}

    /* callbackCompass() //{ */

    void MrsUavFcuApi::publishMagnetometer(const umsg_sensors_mag_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting magnetometer heading");

        if (_capabilities_.produces_magnetometer_heading)
        {
            /*
                        mrs_msgs::Float64Stamped mag_out;
                        mag_out.header.stamp = ros::Time::now();
                        mag_out.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
                        mag_out.value = msg->data;

                        common_handlers_->publishers.publishMagnetometerHeading(mag_out);
                    */
        }
    }

    //}

    /* callbackMagneticField() //{ */

    void MrsUavFcuApi::publishMagneticField(const umsg_sensors_mag_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting magnetic field");

        if (_capabilities_.produces_magnetic_field)
        {

            //            common_handlers_->publishers.publishMagneticField(*msg);
        }
    }

    //}

    /* callbackRC() //{ */

    void MrsUavFcuApi::publishRC(const umsg_control_sBusPacket_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting RC");

        if (_capabilities_.produces_rc_channels)
        {

            /*
            mrs_msgs::HwApiRcChannels rc_out;

            rc_out.stamp = msg->header.stamp;

            for (size_t i = 0; i < msg->channels.size(); i++)
            {
                rc_out.channels.push_back(RCChannelToRange(msg->channels[i]));
            }

            common_handlers_->publishers.publishRcChannels(rc_out);
            */
        }
    }

    //}

    /* callbackAltitude() //{ */

    void MrsUavFcuApi::publishAltitude(const mavros_msgs::Altitude::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting Altitude");

        if (_capabilities_.produces_altitude)
        {

            mrs_msgs::HwApiAltitude altitude_out;

            altitude_out.stamp = msg->header.stamp;
            altitude_out.amsl = msg->amsl;

            common_handlers_->publishers.publishAltitude(altitude_out);
        }
    }

    //}

    /* callbackAltitude() //{ */

    void MrsUavFcuApi::publishGpsStatusRaw(const umsg_sensors_gps_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting Gps Status Raw");

        if (_capabilities_.produces_gnss_status)
        {

            mrs_msgs::GpsInfo gps_info_out;

            gps_info_out.stamp = ser_->FcuToRos(msg.timestamp); // [GPS_FIX_TYPE] GPS fix type
            gps_info_out.fix_type = msg.fixType;                // [GPS_FIX_TYPE] GPS fix type

            gps_info_out.lat = msg.lat; // [deg] Latitude (WGS84, EGM96 ellipsoid)
            gps_info_out.lon = msg.lon;
            gps_info_out.alt = msg.hMSL;   // [m]  (MSL). Positive for up. Not WGS84 altitude.
            gps_info_out.eph = UINT16_MAX; // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
            gps_info_out.epv = UINT16_MAX; // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
            // TODO ASK ABOUT THIS
            gps_info_out.vel = 0; // [m/s] GPS ground speed. If unknown, set to: UINT16_MAX
            gps_info_out.cog = 0;
            // [deg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees.
            gps_info_out.satellites_visible = msg.numSV; // Number of satellites visible. If unknown, set to 255

            gps_info_out.alt_ellipsoid = msg.hELPS; // [m] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
            // TODO ASK ABOUT THIS
            gps_info_out.h_acc = 0;         // [m] Position uncertainty. Positive for up.
            gps_info_out.v_acc = 0;         // [m] Altitude uncertainty. Positive for up.
            gps_info_out.vel_acc = 0;       // [m/s] Speed uncertainty. Positive for up.
            gps_info_out.hdg_acc = 0;       // [deg] Heading / track uncertainty
            gps_info_out.yaw = 0;           // [deg] Yaw in earth frame from north.
            gps_info_out.dgps_num_sats = 0; // Number of DGPS satellites
            gps_info_out.dgps_age = 0;      // [s] Age of DGPS info
            gps_info_out.baseline_dist = 0; // [m] distance to the basestation, not supported by the GPSRAW message

            common_handlers_->publishers.publishGNSSStatus(gps_info_out);
        }
    }

    //}

    /* callbackBattery() //{ */

    void MrsUavFcuApi::publishBattery(const sensor_msgs::BatteryState::ConstPtr msg)
    {

        if (!ser_->isSynced())
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting battery");

        if (_capabilities_.produces_battery_state)
        {

            common_handlers_->publishers.publishBatteryState(*msg);
        }
    }

    //}

    /* callbackGroundTruth() //{ */

    void MrsUavFcuApi::publishGroundTruth(const nav_msgs::Odometry::ConstPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting ground truth");

        auto odom = msg;

        // | ------------------ publish ground truth ------------------ |

        if (_capabilities_.produces_ground_truth)
        {

            nav_msgs::Odometry gt = *msg;

            // if frame_id is "/world", "world", "/map" or "map" gazebo reports velocitites in global world frame so we need to transform them to body frame
            if (msg->header.frame_id == "/world" || msg->header.frame_id == "world" || msg->header.frame_id == "/map" || msg->header.frame_id == "map")
            {

                ROS_INFO_ONCE("[MrsUavFcuApi]: transforming Gazebo ground truth velocities from world to body frame");

                Eigen::Matrix3d R = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);

                Eigen::Vector3d lin_vel_world(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
                Eigen::Vector3d lin_vel_body = R.inverse() * lin_vel_world;

                Eigen::Vector3d angular_vel_world(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
                Eigen::Vector3d angular_vel_body = R.inverse() * angular_vel_world;

                gt.twist.twist.linear.x = lin_vel_body[0];
                gt.twist.twist.linear.y = lin_vel_body[1];
                gt.twist.twist.linear.z = lin_vel_body[2];

                gt.twist.twist.angular.x = angular_vel_body[0];
                gt.twist.twist.angular.y = angular_vel_body[1];
                gt.twist.twist.angular.z = angular_vel_body[2];
            }

            common_handlers_->publishers.publishGroundTruth(gt);
        }

        if (_capabilities_.produces_rtk)
        {
            /*
            double lat;
            double lon;

            mrs_lib::UTMtoLL(msg->pose.pose.position.y + _sim_rtk_utm_y_, msg->pose.pose.position.x + _sim_rtk_utm_x_, _sim_rtk_utm_zone_, lat, lon);

            sensor_msgs::NavSatFix gnss;

            gnss.header.stamp = msg->header.stamp;

            gnss.latitude = lat;
            gnss.longitude = lon;
            gnss.altitude = msg->pose.pose.position.z + _sim_rtk_amsl_;

            mrs_msgs::RtkGps rtk;

            rtk.header.stamp = msg->header.stamp;
            rtk.header.frame_id = "gps";

            rtk.gps.latitude = lat;
            rtk.gps.longitude = lon;
            rtk.gps.altitude = msg->pose.pose.position.z + _sim_rtk_amsl_;
            rtk.gps.covariance[0] = std::pow(0.1, 2);
            rtk.gps.covariance[4] = std::pow(0.1, 2);
            rtk.gps.covariance[8] = std::pow(0.1, 2);

            rtk.fix_type.fix_type = rtk.fix_type.RTK_FIX;

            rtk.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

            common_handlers_->publishers.publishRTK(rtk);
        */
        }
    }

    //}

    /* callbackRTK() //{ */

    void MrsUavFcuApi::publishRTK(const mrs_modules_msgs::Bestpos::ConstPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }

        ROS_INFO_ONCE("[MrsUavFcuApi]: getting rtk");

        mrs_msgs::RtkGps rtk_msg_out;

        rtk_msg_out.gps.latitude = msg->latitude;
        rtk_msg_out.gps.longitude = msg->longitude;
        rtk_msg_out.gps.altitude = msg->height;

        rtk_msg_out.header.stamp = ros::Time::now();
        rtk_msg_out.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

        if (msg->position_type == "L1_INT")
        {
            rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FIX;
        }
        else if (msg->position_type == "L1_FLOAT")
        {
            rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FLOAT;
        }
        else if (msg->position_type == "PSRDIFF")
        {
            rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.DGPS;
        }
        else if (msg->position_type == "SINGLE")
        {
            rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.SPS;
        }
        else if (msg->position_type == "NONE")
        {
            rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.NO_FIX;
        }

        common_handlers_->publishers.publishRTK(rtk_msg_out);
    }

    bool MrsUavFcuApi::ParseMessage(umsg_MessageToTransfer &msg)
    {
        uint8_t msg_class = msg.s.msg_class;
        uint8_t msg_type = msg.s.msg_type;

        bool parsed = true;

        switch (msg_class)
        {
        case UMSG_SENSORS:
        {
            switch (msg_type)
            {
            case SENSORS_IMU:
                break;
            case SENSORS_GPS:
                break;
            case SENSORS_ALTIMETER:
                break;
            case SENSORS_MAG:
                break;

            default:
                parsed = false;
                break;
            }
        }
        break;
        case UMSG_STATE:
        {
            switch (msg_type)
            {
            case STATE_UAV_STATE:
                break;

            default:
                parsed = false;
                break;
            }
        }
        break;

        case UMSG_ESTIMATION:
        {
            switch (msg_type)
            {
            case ESTIMATION_ATTITUDE:
                break;
            case ESTIMATION_POSITION:
                break;

            default:
                parsed = false;
                break;
            }
        }
        break;

        default:
            break;
        }
        return parsed;
    }

    void MrsUavFcuApi::messageParser()
    {

        while (true)
        {
            umsg_MessageToTransfer msg = ser_->waitForPacket();
            bool parsed = false;
            if (_simulation_)
            {
                parsed = hitl_binder_.ParseMessage(msg) || ParseMessage(msg);
            }
            else
            {
                parsed = ParseMessage(msg);
            }

            if (!parsed)
            {
                ROS_ERROR("[HWApi] message class %d and type %d could not be parsed", msg.s.msg_class, msg.s.msg_type);
            }
        }
    };

    // | ------------------------- methods ------------------------ |

    bool MrsUavFcuApi::callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg)
    {
        return false;
    }
    bool MrsUavFcuApi::callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg)
    {
        return false;
    }
    bool MrsUavFcuApi::callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg)
    {
        return false;
    }
    bool MrsUavFcuApi::callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg)
    {
        return false;
    }
    bool MrsUavFcuApi::callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg)
    {
        return false;
    }
    bool MrsUavFcuApi::callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg)
    {
        return false;
    }
    bool MrsUavFcuApi::callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg)
    {
        return false;
    }
    void MrsUavFcuApi::callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg)
    {
        return;
    }
    //}

} // namespace mrs_uav_px4_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_fcu_api::MrsUavFcuApi, mrs_uav_hw_api::MrsUavHwApi)
