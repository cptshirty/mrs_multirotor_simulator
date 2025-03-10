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

#include <mrs_lib/gps_conversions.h>

#include <random>

//}

namespace mrs_hitl_binders
{
    typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;

    /* class MultirotorSimulator //{ */

    class FcuBinder : public nodelet::Nodelet
    {

    public:
        virtual void onInit();

    private:
        ros::NodeHandle nh_;
        std::atomic<bool> is_initialized_;
        // | ------------------------- params ------------------------- |

        ros::Time sim_time_;
        std::mutex mutex_sim_time_;

        // | ------------------------- timers ------------------------- |

        // | ------------------------ rtf check ----------------------- |

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

        std::string uav_name;

        param_loader.loadParam("uav_name", uav_name);

        if (!param_loader.loadedSuccessfully())
        {
            ROS_ERROR("[FcuBinder]: could not load all parameters!");
            ros::shutdown();
        }

        // | ----------------------- publishers ----------------------- |
        // ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>(nh_,uav_name + "/control_group_cmd",1,false);

        // | ----------------------- subscribers ----------------------- |

        // | ----------------------- finish init ---------------------- |
        umsg_CRCInit();
        is_initialized_ = true;
        recvThread_ = std::thread([this]
                                  { this->Receiver(); });

        ROS_INFO("[FcuBinder]: initialized");
    }

    //}

    // | ------------------------- timers ------------------------- |

} // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_hitl_binders::FcuBinder, nodelet::Nodelet)