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
#include <sensor_msgs/MagneticField.h>

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

#include "gps_conversions.h"

#include <random>

//}

#define GRAV_CONST 9.81

namespace mrs_hitl_binders
{
    enum ReceiverState
    {
        WAITING_FOR_SYNC0,
        WAITING_FOR_SYNC1,
        WAITING_FOR_HEADER,
        WAITING_FOR_PAYLOAD,
        INVALID_MSG,
    };
    typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;

    /* class MultirotorSimulator //{ */

    class FcuBinder : public nodelet::Nodelet
    {

    public:
        virtual void onInit();

    private:
        std::mutex serial_mutex_;
        SerialPort ser;
        ros::NodeHandle nh_;
        std::atomic<bool> is_initialized_;
        std::atomic<bool> is_synced_ = false;
        std::string uav_name;
        umsg_MessageToTransfer recvdMsg;
        uint32_t msg_len = 0;
        ReceiverState state = WAITING_FOR_SYNC0;
        // | ------------------------- params ------------------------- |

        ros::Time sim_time_;
        std::mutex mutex_sim_time_;

        std::mutex mutex_sync_time;
        ros::Time sync_time_ROS_send;
        uint32_t sequence_number = 0;

        double startX, startY;
        std::string UTM_zone;

        std::mutex mutex_sync_result;
        std::tuple<ros::Time, uint32_t> sync_result;

        // | ------------------------- timers ------------------------- |

        // | ------------------------ rtf check ----------------------- |

        // | ----------------------- publishers ----------------------- |

        mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd> ph_actuator_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd> ph_control_group_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeRateCmd> ph_attitude_rate_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeCmd> ph_attitude_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> ph_acceleration_hdg_rate_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgCmd> ph_acceleration_hdg_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd> ph_velocity_hdg_rate_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd> ph_velocity_hdg_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::HwApiPositionCmd> ph_position_cmd_;
        mrs_lib::PublisherHandler<mrs_msgs::TrackerCommand> ph_tracker_cmd_;

        mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_pos_est_;

        // | ----------------------- subscribers ----------------------- |

        mrs_lib::SubscribeHandler<sensor_msgs::Imu> sh_imu_;
        mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odom_;
        mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_rangefinder_;
        mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_altitude_;
        mrs_lib::SubscribeHandler<sensor_msgs::MagneticField> sh_mag_;
        mrs_lib::SubscribeHandler<rosgraph_msgs::Clock> sh_clock_;

        mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sh_goto_;

        void callbackOdometry(const nav_msgs::Odometry::ConstPtr msg);
        void callbackIMU(const sensor_msgs::Imu::ConstPtr msg);
        void callbackRangeFinder(const sensor_msgs::Range::ConstPtr msg);
        void callbackAltitude(const nav_msgs::Odometry::ConstPtr msg);
        void callbackMag(const sensor_msgs::MagneticField::ConstPtr msg);
        void callbackTime(const rosgraph_msgs::Clock::ConstPtr msg);
        void callbackGoto(const geometry_msgs::PoseStamped::ConstPtr msg);

        void publishImu(const sensor_msgs::Imu::ConstPtr msg, ros::Time &sim_time);
        void publishMag(const sensor_msgs::MagneticField::ConstPtr msg, ros::Time &sim_time);
        void publishAltitude(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time);
        void publishGps(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time);
        // | ------------------------- system ------------------------- |

        // | -------------------------- time -------------------------- |

        // | -------------------------- Timers -------------------------- |

        std::thread recvThread_;
        void Receiver();
        ros::WallTimer timer_sync_;
        void timerSync(const ros::WallTimerEvent &event);

        // | ------------------------- methods ------------------------ |

        void calculateDelay(umsg_state_heartbeat_t heartbeat);
        uint32_t RosToFcu(ros::Time &rosTime);
        ros::Time FcuToRos(uint32_t &FcuTime);
        void publishPosEst(umsg_estimation_position_t &msg);

        // | --------------- dynamic reconfigure server --------------- |
    };

    //}

    /* onInit() //{ */

    void FcuBinder::onInit()
    {

        is_initialized_ = false;
        nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

        if (!(nh_.hasParam("/use_sim_time")))
        {
            nh_.setParam("/use_sim_time", true);
        }

        srand(time(NULL));

        mrs_lib::ParamLoader param_loader(nh_, "FcuBinder");

        std::string custom_config_path;

        param_loader.loadParam("custom_config", custom_config_path);

        if (custom_config_path != "")
        {
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
        int baud_rate;
        param_loader.loadParam("baud_rate", baud_rate);

        if (!ser.connect(serial_port, baud_rate, false))
        {
            ROS_ERROR("could not open serial port");
            return;
        }
        ser.setBlocking(ser.serial_port_fd_, sizeof(umsg_MessageToTransfer));

        double startLat, startLon;

        param_loader.loadParam("start_latitude", startLat);
        param_loader.loadParam("start_longditude", startLon);

        gps_conversions::LLtoUTM(startLat, startLon, startY, startX, UTM_zone);

        ROS_INFO("SELECTED UTM ZONE IS : %s", UTM_zone.c_str());

        std::string uav_name;

        param_loader.loadParam("uav_name", uav_name);

        if (!param_loader.loadedSuccessfully())
        {
            ROS_ERROR("[FcuBinder]: could not load all parameters!");
            ros::shutdown();
        }

        // | ----------------------- publishers ----------------------- |
        ph_actuator_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>(nh_, "actuators_cmd", 1, false);
        ph_pos_est_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "position_estimation", 1, false);
        // ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>(nh_,uav_name + "/control_group_cmd",1,false);

        // | ----------------------- subscribers ----------------------- |
        mrs_lib::SubscribeHandlerOptions shopts;
        shopts.nh = nh_;
        shopts.node_name = uav_name;
        shopts.no_message_timeout = mrs_lib::no_timeout;
        shopts.threadsafe = true;
        shopts.autostart = true;
        shopts.queue_size = 10;
        shopts.transport_hints = ros::TransportHints().tcpNoDelay();

        sh_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "imu", &FcuBinder::callbackIMU, this);
        sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom", &FcuBinder::callbackOdometry, this);
        sh_rangefinder_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "rangefinder", &FcuBinder::callbackRangeFinder, this);
        sh_altitude_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "altitude", &FcuBinder::callbackAltitude, this);
        sh_mag_ = mrs_lib::SubscribeHandler<sensor_msgs::MagneticField>(shopts, "magnetometer", &FcuBinder::callbackMag, this);
        sh_clock_ = mrs_lib::SubscribeHandler<rosgraph_msgs::Clock>(shopts, "clock_in", &FcuBinder::callbackTime, this);
        sh_goto_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "goal", &FcuBinder::callbackGoto, this);
        timer_sync_ = nh_.createWallTimer(ros::WallDuration(1.00), &FcuBinder::timerSync, this);

        // | ----------------------- finish init ---------------------- |
        umsg_CRCInit();
        is_initialized_ = true;
        recvThread_ = std::thread([this]
                                  { this->Receiver(); });

        ROS_INFO("[FcuBinder]: initialized");
    }

    //}

    // | ------------------------- timers ------------------------- |

    // PublishImu//{
    void FcuBinder::publishImu(const sensor_msgs::Imu::ConstPtr msg, ros::Time &sim_time)
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
        out.s.sensors.imu.timestamp = RosToFcu(sim_time);
        out.s.sensors.imu.temperature = index;
        index += 1;
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_imu_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);
        ser.sendCharArray(out.raw, out.s.len);

    } /*//}*/ /*//}*/

    void FcuBinder::publishMag(const sensor_msgs::MagneticField::ConstPtr msg, ros::Time &sim_time)
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
        out.s.sensors.mag.timestamp = RosToFcu(sim_time);

        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_mag_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);
        ser.sendCharArray(out.raw, out.s.len);
    }

    void FcuBinder::publishAltitude(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time)
    {
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_ALTIMETER;

        out.s.sensors.altimeter.altitude = static_cast<float>(msg->pose.pose.position.z);
        out.s.sensors.altimeter.timestamp = RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_altimeter_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);

        ser.sendCharArray(out.raw, out.s.len);
    }

    void FcuBinder::publishGps(const nav_msgs::Odometry::ConstPtr msg, ros::Time &sim_time)
    {
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_GPS;

        out.s.sensors.gps.timestamp = RosToFcu(sim_time);
        out.s.sensors.gps.fixType = FIX_3D;
        out.s.sensors.gps.hELPS = 0;
        out.s.sensors.gps.hMSL = 0;
        out.s.sensors.gps.reserved = 0;
        out.s.sensors.gps.numSV = 20;

        double UTMNorth, UTMEast;
        UTMEast = startX + msg->pose.pose.position.x;
        UTMNorth = startY + msg->pose.pose.position.y;

        double lat, lon;
        gps_conversions::UTMtoLL(UTMNorth, UTMEast, UTM_zone, lat, lon);

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

        ser.sendCharArray(out.raw, out.s.len);
    }

    // | ------------------------- callbacks ------------------------- |
    void FcuBinder::callbackTime(const rosgraph_msgs::Clock::ConstPtr msg)
    {
        auto sim_time = msg->clock;
        mrs_lib::set_mutexed(mutex_sim_time_, sim_time, sim_time_);
    }

    void FcuBinder::callbackOdometry(const nav_msgs::Odometry::ConstPtr msg)
    {
        if (!is_synced_)
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

        std::scoped_lock lock(serial_mutex_);
        ros::Time sim_time = msg->header.stamp;
        publishGps(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.GPS = 1;
        ROS_INFO_ONCE("[FcuBinder]: GPS CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser.sendCharArray(notifyMsg.raw, notifyMsg.s.len);
    }

    void FcuBinder::callbackIMU(const sensor_msgs::Imu::ConstPtr msg)
    {
        if (!is_synced_)
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

        std::scoped_lock lock(serial_mutex_);

        publishImu(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.imu = 1;
        ROS_INFO_ONCE("[FcuBinder]: IMU CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser.sendCharArray(notifyMsg.raw, notifyMsg.s.len);
        // ROS_INFO("[FcuBinder]: IMU Duration %d",diff_to_now.toNSec());
        //  toto send the message over the serial
    }

    void FcuBinder::callbackGoto(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        if (!is_synced_)
        {
            return;
        }
        // counts on the origin being the same as the sim origin
        // fill the packet header
        ros::Time sim_time = msg->header.stamp;

        umsg_MessageToTransfer GoalMsg;
        GoalMsg.s.sync0 = 'M';
        GoalMsg.s.sync1 = 'R';
        GoalMsg.s.msg_class = UMSG_AUTONOMY;
        GoalMsg.s.msg_type = AUTONOMY_POSITIONGOAL;
        GoalMsg.s.autonomy.PositionGoal.goal[0] = msg->pose.position.x;
        GoalMsg.s.autonomy.PositionGoal.goal[1] = msg->pose.position.y;
        GoalMsg.s.autonomy.PositionGoal.goal[2] = 10.0;
        GoalMsg.s.autonomy.PositionGoal.velocity = 4;
        GoalMsg.s.autonomy.PositionGoal.timestamp = RosToFcu(sim_time);

        std::scoped_lock lock(serial_mutex_);

        ROS_INFO("[FcuBinder]: GOAL CALLBACK called");

        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_autonomy_PositionGoal_t) + 1;
        GoalMsg.s.len = len;
        GoalMsg.raw[len - 1] = umsg_calcCRC(GoalMsg.raw, len - 1);
        ser.sendCharArray(GoalMsg.raw, GoalMsg.s.len);
    }

    void FcuBinder::callbackRangeFinder(const sensor_msgs::Range::ConstPtr msg)
    {
        ROS_WARN_ONCE("rangefinder callback not yet implemented");
    }

    void FcuBinder::callbackAltitude(const nav_msgs::Odometry::ConstPtr msg)
    {
        if (!is_synced_)
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

        std::scoped_lock lock(serial_mutex_);

        publishAltitude(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.altimeter = 1;
        ROS_INFO_ONCE("[FcuBinder]: Altitude CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser.sendCharArray(notifyMsg.raw, notifyMsg.s.len);
        // ROS_INFO("[FcuBinder]: IMU Duration %d",diff_to_now.toNSec());
        //  toto send the message over the serial
    }
    void FcuBinder::callbackMag(const sensor_msgs::MagneticField::ConstPtr msg)
    {
        if (!is_synced_)
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

        std::scoped_lock lock(serial_mutex_);
        publishMag(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.magnetometer = 1;
        ROS_INFO_ONCE("[FcuBinder]: mag CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser.sendCharArray(notifyMsg.raw, notifyMsg.s.len);
        // ROS_INFO("[FcuBinder]: IMU Duration %d",diff_to_now.toNSec());
        //  toto send the message over the serial
    }

    void FcuBinder::Receiver()
    {
        state = WAITING_FOR_SYNC0;
        bool receptionComplete = false;
        ROS_INFO_ONCE("[FcuBinder]: ReceiverActive spinning");
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
                // ROS_INFO("[FcuBinder]: received message of class %d and type %d", recvdMsg.s.msg_class, recvdMsg.s.msg_type);
                if (recvdMsg.s.msg_class == UMSG_CONTROL && recvdMsg.s.msg_type == CONTROL_DSHOTMESSAGE)
                {

                    mrs_msgs::HwApiActuatorCmd cmd;
                    umsg_control_DshotMessage_t DshotMessage = recvdMsg.s.control.DshotMessage;
                    cmd.stamp = FcuToRos(DshotMessage.timestamp);

                    for (size_t i = 0; i < 4; i++)
                    {
                        cmd.motors.push_back(static_cast<float>(DshotMessage.channels[i]) / 2048.);
                    }
                    ph_actuator_cmd_.publish(cmd);
                }
                else if (recvdMsg.s.msg_class == UMSG_STATE && recvdMsg.s.msg_type == STATE_HEARTBEAT)
                {

                    umsg_state_heartbeat_t beat = recvdMsg.s.state.heartbeat;

                    calculateDelay(beat);
                    if (!is_synced_)
                    {
                        calculateDelay(beat);
                        is_synced_ = true;
                    }
                }
                else if (recvdMsg.s.msg_class == UMSG_ESTIMATION && recvdMsg.s.msg_type == ESTIMATION_POSITION)
                {
                    ROS_INFO_ONCE("publishing the position estimate");
                    umsg_estimation_position_t pos_est = recvdMsg.s.estimation.position;
                    publishPosEst(pos_est);
                }
                else
                {
                    ROS_WARN("received msg of class %d and type %d", recvdMsg.s.msg_class, recvdMsg.s.msg_type);
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

    void FcuBinder::timerSync(const ros::WallTimerEvent &event)
    {

        if (!is_initialized_)
        {
            return;
        }
        ROS_INFO_ONCE("[FcuBinder]: sync timer spinning");

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

        {
            std::scoped_lock lock(serial_mutex_);
            ser.sendCharArray(msg.raw, msg.s.len);
        }

        // auto curr_time   = ros::time::now();
        mrs_lib::set_mutexed(mutex_sync_time, std::tuple(curr_time, sequential), std::forward_as_tuple(sync_time_ROS_send, sequence_number));

        return;
    }

    // | ------------------------- methods ------------------------ |

    void FcuBinder::calculateDelay(umsg_state_heartbeat_t heartbeat)
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

    uint32_t FcuBinder::RosToFcu(ros::Time &rosTime)
    {

        auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
        int64_t diff = (rosTime.toNSec() - syncTime_R.toNSec()) / 1e6;
        int64_t new_stamp = diff + static_cast<int64_t>(syncTime_F);
        return static_cast<uint32_t>(new_stamp);
    }

    ros::Time FcuBinder::FcuToRos(uint32_t &FcuTime)
    {
        auto [syncTime_R, syncTime_F] = mrs_lib::get_mutexed(mutex_sync_result, sync_result);
        int64_t diff = static_cast<int64_t>(FcuTime) - static_cast<int64_t>(syncTime_F) * 1e6;

        ros::Duration diff_R;
        diff_R.fromNSec(diff);

        return syncTime_R + diff_R;
    }

    void FcuBinder::publishPosEst(umsg_estimation_position_t &msg)
    {
        nav_msgs::Odometry odom_est;
        odom_est.header.stamp = FcuToRos(msg.timestamp);
        odom_est.pose.pose.position.x = msg.position[0];
        odom_est.pose.pose.position.y = msg.position[1];
        odom_est.pose.pose.position.z = msg.position[2];

        odom_est.twist.twist.linear.x = msg.velocity[0];
        odom_est.twist.twist.linear.y = msg.velocity[1];
        odom_est.twist.twist.linear.z = msg.velocity[2];
        ph_pos_est_.publish(odom_est);
    }
} // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_hitl_binders::FcuBinder, nodelet::Nodelet)