#include <uav_system_ros.h>
#include <cmath>

namespace mrs_multirotor_simulator
{

    /* UavSystemRos //{ */

    UavSystemRos::UavSystemRos(ros::NodeHandle &nh, const std::string uav_name)
    {

        time_last_input_ = ros::Time(0);

        _uav_name_ = uav_name;

        mrs_lib::ParamLoader param_loader(nh, "UavSystemRos");

        std::string custom_config_path;

        param_loader.loadParam("custom_config", custom_config_path);

        if (custom_config_path != "")
        {
            param_loader.addYamlFile(custom_config_path);
        }

        param_loader.addYamlFileFromParam("config");
        param_loader.addYamlFileFromParam("config_uavs");

        std::string type;
        param_loader.loadParam(uav_name + "/type", type);

        // | --------------------- general params --------------------- |

        param_loader.loadParam("frames/world/name", _frame_world_);
        bool prefix_world_name;
        param_loader.loadParam("frames/world/prefix_with_uav_name", prefix_world_name);

        if (prefix_world_name)
        {
            _frame_world_ = uav_name + "/" + _frame_world_;
        }

        param_loader.loadParam("frames/rangefinder/name", _frame_rangefinder_);

        _frame_rangefinder_ = uav_name + "/" + _frame_rangefinder_;

        param_loader.loadParam("frames/rangefinder/publish_tf", _publish_rangefinder_tf_);
        param_loader.loadParam("frames/fcu/publish_tf", _publish_fcu_tf_);

        param_loader.loadParam("frames/fcu/name", _frame_fcu_);

        _frame_fcu_ = uav_name + "/" + _frame_fcu_;

        param_loader.loadParam("g", model_params_.g);
        param_loader.loadParam("iterate_without_input", _iterate_without_input_);
        param_loader.loadParam("input_timeout", _input_timeout_);
        param_loader.loadParam("ground/enabled", model_params_.ground_enabled);
        param_loader.loadParam("ground/z", model_params_.ground_z);
        param_loader.loadParam("individual_takeoff_platform/enabled", model_params_.takeoff_patch_enabled);

        // | ------------------ model-specific params ----------------- |

        param_loader.loadParam(type + "/n_motors", model_params_.n_motors);
        param_loader.loadParam(type + "/mass", model_params_.mass);
        param_loader.loadParam(type + "/arm_length", model_params_.arm_length);
        param_loader.loadParam(type + "/body_height", model_params_.body_height);
        param_loader.loadParam(type + "/air_resistance_coeff", model_params_.air_resistance_coeff);
        param_loader.loadParam(type + "/motor_time_constant", model_params_.motor_time_constant);
        param_loader.loadParam(type + "/propulsion/prop_radius", model_params_.prop_radius);
        param_loader.loadParam(type + "/propulsion/force_constant", model_params_.kf);
        param_loader.loadParam(type + "/propulsion/moment_constant", model_params_.km);
        param_loader.loadParam(type + "/propulsion/rpm/min", model_params_.min_rpm);
        param_loader.loadParam(type + "/propulsion/rpm/max", model_params_.max_rpm);

        // | --------------------- spawn location --------------------- |

        double spawn_x;
        double spawn_y;
        double spawn_z;
        double spawn_heading;

        param_loader.loadParam(uav_name + "/spawn/x", spawn_x);
        param_loader.loadParam(uav_name + "/spawn/y", spawn_y);
        param_loader.loadParam(uav_name + "/spawn/z", spawn_z);
        param_loader.loadParam(uav_name + "/spawn/heading", spawn_heading);

        param_loader.loadParam("randomization/enabled", _randomization_enabled_);
        param_loader.loadParam("randomization/bounds/x", _randomization_bounds_x_);
        param_loader.loadParam("randomization/bounds/y", _randomization_bounds_y_);
        param_loader.loadParam("randomization/bounds/z", _randomization_bounds_z_);

        if (_randomization_enabled_)
        {
            spawn_x += randd(-_randomization_bounds_x_, _randomization_bounds_x_);
            spawn_y += randd(-_randomization_bounds_y_, _randomization_bounds_y_);
            spawn_z += randd(-_randomization_bounds_z_, _randomization_bounds_z_);
            spawn_heading += randd(-3.14, 3.14);
        }

        calculateInertia(model_params_);

        model_params_.allocation_matrix = param_loader.loadMatrixDynamic2(type + "/propulsion/allocation_matrix", 4, -1);

        model_params_.allocation_matrix.row(0) *= model_params_.arm_length * model_params_.kf;
        model_params_.allocation_matrix.row(1) *= model_params_.arm_length * model_params_.kf;
        model_params_.allocation_matrix.row(2) *= model_params_.km * (3.0 * model_params_.prop_radius) * model_params_.kf;
        model_params_.allocation_matrix.row(3) *= model_params_.kf;

        uav_system_ = UavSystem(model_params_, Eigen::Vector3d(spawn_x, spawn_y, spawn_z), spawn_heading);

        // | -------------------------- mixer ------------------------- |

        Mixer::Params mixer_params;

        param_loader.loadParam("mixer/desaturation", mixer_params.desaturation);

        uav_system_.setMixerParams(mixer_params);

        // | --------------------- rate controller -------------------- |

        RateController::Params rate_controller_params;

        param_loader.loadParam("rate_controller/kp", rate_controller_params.kp);
        param_loader.loadParam("rate_controller/kd", rate_controller_params.kd);
        param_loader.loadParam("rate_controller/ki", rate_controller_params.ki);

        uav_system_.setRateControllerParams(rate_controller_params);

        // | --------------------- attitude controller -------------------- |

        AttitudeController::Params attitude_controller_params;

        param_loader.loadParam("attitude_controller/kp", attitude_controller_params.kp);
        param_loader.loadParam("attitude_controller/kd", attitude_controller_params.kd);
        param_loader.loadParam("attitude_controller/ki", attitude_controller_params.ki);
        param_loader.loadParam("attitude_controller/max_rate_roll_pitch", attitude_controller_params.max_rate_roll_pitch);
        param_loader.loadParam("attitude_controller/max_rate_yaw", attitude_controller_params.max_rate_yaw);

        uav_system_.setAttitudeControllerParams(attitude_controller_params);

        // | ------------------- velocity controller ------------------ |

        VelocityController::Params velocity_controller_params;

        param_loader.loadParam("velocity_controller/kp", velocity_controller_params.kp);
        param_loader.loadParam("velocity_controller/kd", velocity_controller_params.kd);
        param_loader.loadParam("velocity_controller/ki", velocity_controller_params.ki);
        param_loader.loadParam("velocity_controller/max_acceleration", velocity_controller_params.max_acceleration);

        uav_system_.setVelocityControllerParams(velocity_controller_params);

        // | ------------------- position controller ------------------ |

        PositionController::Params position_controller_params;

        param_loader.loadParam("position_controller/kp", position_controller_params.kp);
        param_loader.loadParam("position_controller/kd", position_controller_params.kd);
        param_loader.loadParam("position_controller/ki", position_controller_params.ki);
        param_loader.loadParam("position_controller/max_velocity", position_controller_params.max_velocity);

        uav_system_.setPositionControllerParams(position_controller_params);

        // | ----------------------- noise generation ---------------------- |
        double bias = 0;
        double stddev = 0;

        // accel
        param_loader.loadParam("accel_bias", bias);
        param_loader.loadParam("accel_stddev", stddev);
        accel_gen_ = std::normal_distribution<double>(bias, stddev);

        // gyro
        param_loader.loadParam("gyro_bias", bias);
        param_loader.loadParam("gyro_stddev", stddev);
        gyro_gen_ = std::normal_distribution<double>(bias, stddev);

        // altitude
        param_loader.loadParam("altitude_bias", bias);
        param_loader.loadParam("altitude_stddev", stddev);
        altitude_gen_ = std::normal_distribution<double>(bias, stddev);

        // mag
        param_loader.loadParam("mag_bias", bias);
        param_loader.loadParam("mag_stddev", stddev);
        mag_gen_ = std::normal_distribution<double>(bias, stddev);

        // position
        param_loader.loadParam("pos_bias", bias);
        param_loader.loadParam("pos_stddev", stddev);
        position_gen_ = std::normal_distribution<double>(bias, stddev);

        // range
        param_loader.loadParam("range_bias", bias);
        param_loader.loadParam("range_stddev", stddev);

        range_gen_ = std::normal_distribution<double>(bias, stddev);

        // load the filters into std vector
        std::vector<double> b_coeffs;
        std::vector<double> a_coeffs;
        a_coeffs.push_back(1);

        // accel
        const std::string base_accel = "B_accel";

        for (int i = 0; i < 3; i++)
        {
            std::string param = base_accel + std::to_string(i);
            param_loader.loadParam(param, b_coeffs);
            accel_noiseShapers_.push_back(mrs_lib::IirFilter(a_coeffs, b_coeffs));
        }

        // gyro
        const std::string base_gyro = "B_gyro";

        for (int i = 0; i < 3; i++)
        {
            std::string param = base_gyro + std::to_string(i);
            param_loader.loadParam(param, b_coeffs);
            gyro_noiseShapers_.push_back(mrs_lib::IirFilter(a_coeffs, b_coeffs));
        }

        // altitude
        param_loader.loadParam("B_altitude", b_coeffs);
        altitude_noiseShaper_ = mrs_lib::IirFilter(a_coeffs, b_coeffs);

        // mag
        const std::string base_mag = "B_mag";

        for (int i = 0; i < 3; i++)
        {
            std::string param = base_mag + std::to_string(i);
            param_loader.loadParam(param, b_coeffs);
            mag_noiseShapers_.push_back(mrs_lib::IirFilter(a_coeffs, b_coeffs));
        }

        // position
        const std::string base_pos = "B_position";

        for (int i = 0; i < 3; i++)
        {
            std::string param = base_pos + std::to_string(i);
            param_loader.loadParam(param, b_coeffs);
            position_noiseShapers_.push_back(mrs_lib::IirFilter(a_coeffs, b_coeffs));
        }

        // range
        param_loader.loadParam("B_range", b_coeffs);
        range_noiseShaper_ = mrs_lib::IirFilter(a_coeffs, b_coeffs);

        // | ----------------------- load the rates for publishing ----------------------- |

        double frequency = 0;

        param_loader.loadParam("imu_rate", frequency);
        imu_delay_.fromNSec(static_cast<int64_t>(1e9 / frequency));

        param_loader.loadParam("mag_rate", frequency);
        mag_delay_.fromNSec(static_cast<int64_t>(1e9 / frequency));

        param_loader.loadParam("altitude_rate", frequency);
        altitude_delay_.fromNSec(static_cast<int64_t>(1e9 / frequency));

        param_loader.loadParam("position_rate", frequency);
        position_delay_.fromNSec(static_cast<int64_t>(1e9 / frequency));

        param_loader.loadParam("range_rate", frequency);
        range_delay_.fromNSec(static_cast<int64_t>(1e9 / frequency));

        if (!param_loader.loadedSuccessfully())
        {
            ROS_ERROR("[%s]: failed to load all parameters", _uav_name_.c_str());
            ros::shutdown();
        }

        // | ----------------------- publishers ----------------------- |

        ph_imu_ = mrs_lib::PublisherHandler<sensor_msgs::Imu>(nh, uav_name + "/imu", 1, false);
        ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, uav_name + "/odom", 1, false);
        ph_rangefinder_ = mrs_lib::PublisherHandler<sensor_msgs::Range>(nh, uav_name + "/rangefinder", 1, false);
        ph_altitude_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, uav_name + "/altitude", 1, false); // only the z value is valid
        ph_mag_ = mrs_lib::PublisherHandler<sensor_msgs::MagneticField>(nh, uav_name + "/magnetometer", 1, false);

        ph_imu_noise_ = mrs_lib::PublisherHandler<sensor_msgs::Imu>(nh, uav_name + "/imu_noise", 1, false);
        ph_odom_noise_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, uav_name + "/odom_noise", 1, false);
        ph_rangefinder_noise_ = mrs_lib::PublisherHandler<sensor_msgs::Range>(nh, uav_name + "/rangefinder_noise", 1, false);
        ph_altitude_noise_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, uav_name + "/altitude_noise", 1, false); // only the z value is valid
        ph_mag_noise_ = mrs_lib::PublisherHandler<sensor_msgs::MagneticField>(nh, uav_name + "/magnetometer_noise", 1, false);

        // | ----------------------- subscribers ---------------------- |

        mrs_lib::SubscribeHandlerOptions shopts;
        shopts.nh = nh;
        shopts.node_name = _uav_name_;
        shopts.no_message_timeout = mrs_lib::no_timeout;
        shopts.threadsafe = true;
        shopts.autostart = true;
        shopts.queue_size = 10;
        shopts.transport_hints = ros::TransportHints().tcpNoDelay();

        sh_actuator_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>(shopts, uav_name + "/actuators_cmd", &UavSystemRos::callbackActuatorCmd, this);

        sh_control_group_cmd_ =
            mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>(shopts, uav_name + "/control_group_cmd", &UavSystemRos::callbackControlGroupCmd, this);

        sh_attitude_rate_cmd_ =
            mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>(shopts, uav_name + "/attitude_rate_cmd", &UavSystemRos::callbackAttitudeRateCmd, this);

        sh_attitude_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>(shopts, uav_name + "/attitude_cmd", &UavSystemRos::callbackAttitudeCmd, this);

        sh_acceleration_hdg_cmd_ =
            mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>(shopts, uav_name + "/acceleration_hdg_cmd", &UavSystemRos::callbackAccelerationHdgCmd, this);

        sh_acceleration_hdg_rate_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd>(shopts, uav_name + "/acceleration_hdg_rate_cmd",
                                                                                                         &UavSystemRos::callbackAccelerationHdgRateCmd, this);

        sh_velocity_hdg_cmd_ =
            mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>(shopts, uav_name + "/velocity_hdg_cmd", &UavSystemRos::callbackVelocityHdgCmd, this);

        sh_velocity_hdg_rate_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>(shopts, uav_name + "/velocity_hdg_rate_cmd",
                                                                                                 &UavSystemRos::callbackVelocityHdgRateCmd, this);

        sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>(shopts, uav_name + "/position_cmd", &UavSystemRos::callbackPositionCmd, this);

        sh_tracker_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, uav_name + "/tracker_cmd", &UavSystemRos::callbackTrackerCmd, this);

        // | --------------------- tf broadcaster --------------------- |

        tf_broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>();

        // | --------------------- service servers -------------------- |

        service_server_set_mass_ = nh.advertiseService(uav_name + "/set_mass", &UavSystemRos::callbackSetMass, this);

        service_server_set_ground_z_ = nh.advertiseService(uav_name + "/set_ground_z", &UavSystemRos::callbackSetGroundZ, this);

        // | ------------------ first model iteration ----------------- |

        // * we need to iterate the model first to initialize its state
        // * this needs to happen in order to publish the correct state
        //   when using iterate_without_input == false

        reference::Actuators actuators_cmd;

        actuators_cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

        // set the motor input for the model
        uav_system_.setInput(actuators_cmd);

        // iterate the model twise to initialize all the states
        uav_system_.makeStep(0.01);
        uav_system_.makeStep(0.01);

        is_initialized_ = true;

        ROS_INFO("[%s]: initialized", _uav_name_.c_str());
    }

    //}

    /* makeStep() //{ */

    void UavSystemRos::makeStep(const double dt, const ros::Time &sim_time)
    {

        // | ---------------- check timeout of an input --------------- |

        auto time_last_input = mrs_lib::get_mutexed(mutex_time_last_input_, time_last_input_);

        if (time_last_input > ros::Time(0))
        {
            if ((ros::Time::now() - time_last_input).toSec() > _input_timeout_)
            {

                ROS_WARN("[%s]: input timeouted", _uav_name_.c_str());

                timeoutInput();

                {
                    std::scoped_lock lock(mutex_time_last_input_);
                    time_last_input_ = ros::Time(0);
                }
            }
        }

        // | --------------------- model iteration -------------------- |

        if (_iterate_without_input_ || time_last_input_ > ros::Time(0))
        {

            std::scoped_lock lock(mutex_uav_system_);

            // iterate the model
            uav_system_.makeStep(dt);
        }

        // extract the current state
        MultirotorModel::State state = uav_system_.getState();

        // publish data

        // position
        publishOdometry(state, sim_time);

        // imu
        publishIMU(state, sim_time);

        // rangefinder
        publishRangefinder(state, sim_time);

        // mag
        publishMag(state, sim_time);

        // altimeter
        publishAltitude(state, sim_time);

        if (_publish_fcu_tf_)
        {

            geometry_msgs::TransformStamped tf;

            tf.header.stamp = ros::Time::now();
            tf.header.frame_id = _frame_world_;
            tf.child_frame_id = _frame_fcu_;

            tf.transform.translation.x = state.x(0);
            tf.transform.translation.y = state.x(1);
            tf.transform.translation.z = state.x(2);

            tf.transform.rotation = mrs_lib::AttitudeConverter(state.R);

            tf_broadcaster_->sendTransform(tf);
        }
    }

    //}

    /* getPose() //{ */

    Eigen::Vector3d UavSystemRos::getPose(void)
    {

        return uav_system_.getState().x;
    }

    //}

    /* getParams() //{ */

    MultirotorModel::ModelParams UavSystemRos::getParams()
    {

        return uav_system_.getParams();
    }

    //}

    /* getState() //{ */

    MultirotorModel::State UavSystemRos::getState()
    {

        return uav_system_.getState();
    }

    //}

    /* crash() //{ */

    void UavSystemRos::crash(void)
    {
        uav_system_.crash();
    }

    //}

    /* hasCrashed() //{ */

    bool UavSystemRos::hasCrashed(void)
    {
        return uav_system_.hasCrashed();
    }

    //}

    /* applyForce() //{ */

    void UavSystemRos::applyForce(const Eigen::Vector3d &force)
    {
        uav_system_.applyForce(force);
    }

    //}

    // | ----------------------- publishers ----------------------- |

    /* publishOdometry() //{ */

    void UavSystemRos::publishOdometry(const MultirotorModel::State &state, const ros::Time &sim_time)
    {

        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = _frame_world_;
        odom.child_frame_id = _frame_fcu_;

        odom.pose.pose.orientation = mrs_lib::AttitudeConverter(state.R);

        odom.pose.pose.position.x = state.x(0);
        odom.pose.pose.position.y = state.x(1);
        odom.pose.pose.position.z = state.x(2);

        Eigen::Vector3d vel_body = state.R.transpose() * state.v;

        odom.twist.twist.linear.x = vel_body(0);
        odom.twist.twist.linear.y = vel_body(1);
        odom.twist.twist.linear.z = vel_body(2);

        odom.twist.twist.angular.x = state.omega(0);
        odom.twist.twist.angular.y = state.omega(1);
        odom.twist.twist.angular.z = state.omega(2);

        ph_odom_.publish(odom);
        // add the noise

        if (sim_time - position_last_stamp_ >= position_delay_)
        {

            odom.pose.pose.position.x += position_noiseShapers_.at(0).iterate(position_gen_(gen));
            odom.pose.pose.position.y += position_noiseShapers_.at(1).iterate(position_gen_(gen));
            odom.pose.pose.position.z += position_noiseShapers_.at(2).iterate(position_gen_(gen));

            ph_odom_noise_.publish(odom);
            position_last_stamp_ = sim_time;
        }
    }

    //}

    /* publishIMU() //{ */

    void UavSystemRos::publishIMU(const MultirotorModel::State &state, const ros::Time &sim_time)
    {

        sensor_msgs::Imu imu;

        imu.header.stamp = sim_time;
        imu.header.frame_id = _frame_fcu_;

        imu.angular_velocity.x = state.omega(0);
        imu.angular_velocity.y = state.omega(1);
        imu.angular_velocity.z = state.omega(2);

        auto acc = uav_system_.getImuAcceleration();

        imu.linear_acceleration.x = acc(0);
        imu.linear_acceleration.y = acc(1);
        imu.linear_acceleration.z = acc(2);

        imu.orientation = mrs_lib::AttitudeConverter(state.R);

        ph_imu_.publish(imu);

        if (sim_time - imu_last_stamp_ >= imu_delay_)
        {
            imu_last_stamp_ = sim_time;
            // add the noise
            imu.angular_velocity.x += gyro_noiseShapers_.at(0).iterate(gyro_gen_(gen));
            imu.angular_velocity.y += gyro_noiseShapers_.at(1).iterate(gyro_gen_(gen));
            imu.angular_velocity.z += gyro_noiseShapers_.at(2).iterate(gyro_gen_(gen));

            imu.linear_acceleration.x += accel_noiseShapers_.at(0).iterate(accel_gen_(gen));
            imu.linear_acceleration.y += accel_noiseShapers_.at(1).iterate(accel_gen_(gen));
            imu.linear_acceleration.z += accel_noiseShapers_.at(2).iterate(accel_gen_(gen));

            ph_imu_noise_.publish(imu);
        }
    }

    //}

    /* publishRangefinder() //{ */

    void UavSystemRos::publishRangefinder(const MultirotorModel::State &state, const ros::Time &sim_time)
    {

        // | ----------------------- publish tf ----------------------- |

        const Eigen::Vector3d body_z = state.R.col(2);
        const Eigen::Vector3d rangefinder_dir = -body_z;

        // calculate the angle between the drone's z axis and the world's z axis
        double tilt = acos(rangefinder_dir.dot(Eigen::Vector3d(0, 0, -1)));

        double range_measurement;

        if (body_z(2) > 0)
        {
            range_measurement = (state.x(2) - model_params_.ground_z) / cos(tilt) + 0.01;
        }
        else
        {
            range_measurement = std::numeric_limits<double>::max();
        }

        if (range_measurement > 40.0)
        {
            range_measurement = 41.0;
        }

        sensor_msgs::Range range;

        range.header.frame_id = _frame_rangefinder_;
        range.header.stamp = sim_time;
        range.max_range = 40.0;
        range.min_range = 0.0;
        range.range = range_measurement;
        range.radiation_type = range.INFRARED;
        range.field_of_view = 0.01;

        ph_rangefinder_.publish(range);

        // add the noise
        if (sim_time - range_last_stamp_ >= range_delay_)
        {
            range.range += range_noiseShaper_.iterate(range_gen_(gen));
            ph_rangefinder_noise_.publish(range);
            range_last_stamp_ = sim_time;
        }

        if (_publish_rangefinder_tf_)
        {

            geometry_msgs::TransformStamped tf;

            tf.header.stamp = sim_time;
            tf.header.frame_id = _frame_fcu_;
            tf.child_frame_id = _frame_rangefinder_;

            tf.transform.translation.x = 0;
            tf.transform.translation.y = 0;
            tf.transform.translation.z = -0.05;

            tf.transform.rotation = mrs_lib::AttitudeConverter(0, 1.57, 0);

            tf_broadcaster_->sendTransform(tf);
        }
    }

    //}

    /* publishAltitude() //{ */

    void UavSystemRos::publishAltitude(const MultirotorModel::State &state, const ros::Time &sim_time)
    {

        nav_msgs::Odometry odom;

        odom.header.stamp = sim_time;
        odom.header.frame_id = _frame_world_;
        odom.child_frame_id = _frame_fcu_;

        odom.pose.pose.orientation = mrs_lib::AttitudeConverter(state.R);

        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = state.x(2);

        ph_altitude_.publish(odom);
        // add the noise

        if (sim_time - altitude_last_stamp_ >= altitude_delay_)
        {
            odom.pose.pose.position.z += altitude_noiseShaper_.iterate(altitude_gen_(gen));

            ph_altitude_noise_.publish(odom);
            altitude_last_stamp_ = sim_time;
        }
    }

    //}

    /* publishMag() //{ */

    void UavSystemRos::publishMag(const MultirotorModel::State &state, const ros::Time &sim_time)
    {

        // TODO implement this

        sensor_msgs::MagneticField mag;

        mag.header.stamp = sim_time;
        mag.header.frame_id = _frame_fcu_;

        Eigen::Vector3d vec_north(0, 1, 0); // vector pointing to north in ENU frame
        Eigen::Vector3d field = state.R.inverse() * vec_north;
        mag.magnetic_field.x = field.x();
        mag.magnetic_field.y = field.y();
        mag.magnetic_field.z = field.z();

        ph_mag_.publish(mag);
        // add the noise

        if (sim_time - mag_last_stamp_ >= mag_delay_)
        {
            mag.magnetic_field.x += mag_noiseShapers_.at(0).iterate(mag_gen_(gen));
            mag.magnetic_field.y += mag_noiseShapers_.at(1).iterate(mag_gen_(gen));
            mag.magnetic_field.z += mag_noiseShapers_.at(2).iterate(mag_gen_(gen));

            // TODO add the noise to the magnetometer

            ph_mag_noise_.publish(mag);
            mag_last_stamp_ = sim_time;
        }
    }

    //}

    // | ------------------------ routines ------------------------ |

    /* timeoutInput() //{ */

    void UavSystemRos::timeoutInput(void)
    {

        auto last_input_mode = mrs_lib::get_mutexed(mutex_time_last_input_, last_input_mode_);

        MultirotorModel::State state = uav_system_.getState();

        switch (last_input_mode)
        {

        case UavSystem::POSITION_CMD:
        {

            reference::Position cmd;

            cmd.position = state.x;
            cmd.heading = mrs_lib::AttitudeConverter(state.R).getHeading();

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::VELOCITY_HDG_CMD:
        {

            reference::VelocityHdg cmd;

            cmd.velocity = Eigen::Vector3d(0, 0, 0);
            cmd.heading = mrs_lib::AttitudeConverter(state.R).getHeading();

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::VELOCITY_HDG_RATE_CMD:
        {

            reference::VelocityHdgRate cmd;

            cmd.velocity = Eigen::Vector3d(0, 0, 0);
            cmd.heading_rate = 0;

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::ACCELERATION_HDG_CMD:
        {

            reference::AccelerationHdg cmd;

            cmd.acceleration = Eigen::Vector3d(0, 0, 0);
            cmd.heading = mrs_lib::AttitudeConverter(state.R).getHeading();

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::ACCELERATION_HDG_RATE_CMD:
        {

            reference::AccelerationHdgRate cmd;

            cmd.acceleration = Eigen::Vector3d(0, 0, 0);
            cmd.heading_rate = 0;

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::ATTITUDE_CMD:
        {

            reference::Attitude cmd;

            double heading = mrs_lib::AttitudeConverter(state.R).getHeading();

            cmd.orientation = mrs_lib::AttitudeConverter(0, 0, heading);
            cmd.throttle = 0.0;

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::TILT_HDG_RATE_CMD:
        {

            reference::TiltHdgRate cmd;

            cmd.tilt_vector = Eigen::Vector3d(0, 0, 1);
            cmd.throttle = 0.0;

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::ATTITUDE_RATE_CMD:
        {

            reference::AttitudeRate cmd;

            cmd.rate_x = 0;
            cmd.rate_y = 0;
            cmd.rate_z = 0;
            cmd.throttle = 0.0;

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::CONTROL_GROUP_CMD:
        {

            reference::ControlGroup cmd;

            cmd.roll = 0;
            cmd.pitch = 0;
            cmd.yaw = 0;
            cmd.throttle = 0.0;

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::ACTUATOR_CMD:
        {

            reference::Actuators cmd;

            cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput(cmd);
            }

            break;
        }

        case UavSystem::INPUT_UNKNOWN:
        {

            {
                std::scoped_lock lock(mutex_uav_system_);
                uav_system_.setInput();
            }

            break;
        }
        }
    }

    //}

    /* randd() //{ */

    double UavSystemRos::randd(double from, double to)
    {

        double zero_to_one = double((float)rand()) / double(RAND_MAX);

        return floor(to - from) * zero_to_one + from;
    }

    //}

    /* calculateInertia() //{ */

    void UavSystemRos::calculateInertia(MultirotorModel::ModelParams &params)
    {

        // create the inertia matrix
        params.J = Eigen::Matrix3d::Zero();
        params.J(0, 0) = params.mass * (3.0 * params.arm_length * params.arm_length + params.body_height * params.body_height) / 12.0;
        params.J(1, 1) = params.mass * (3.0 * params.arm_length * params.arm_length + params.body_height * params.body_height) / 12.0;
        params.J(2, 2) = (params.mass * params.arm_length * params.arm_length) / 2.0;
    }

    //}

    // | ------------------------ callbacks ----------------------- |

    /* callbackActuatorCmd() //{ */

    void UavSystemRos::callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting actuators command", _uav_name_.c_str());

        if (model_params_.n_motors != int(msg->motors.size()))
        {
            ROS_ERROR("[%s]: the actuators message controls %d motors, but the model has %d motors", _uav_name_.c_str(), int(msg->motors.size()),
                      model_params_.n_motors);
            return;
        }

        reference::Actuators cmd;

        cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

        for (int i = 0; i < model_params_.n_motors; i++)
        {
            cmd.motors(i) = msg->motors.at(i);
        }

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::ACTUATOR_CMD;
        }
    }

    //}

    /* callbackControlGroupCmd() //{ */

    void UavSystemRos::callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting control group command", _uav_name_.c_str());

        reference::ControlGroup cmd;

        cmd.throttle = msg->throttle;
        cmd.roll = msg->roll;
        cmd.pitch = msg->pitch;
        cmd.yaw = msg->yaw;

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::CONTROL_GROUP_CMD;
        }
    }

    //}

    /* callbackAttitudeRateCmd() //{ */

    void UavSystemRos::callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting attitude rate command", _uav_name_.c_str());

        reference::AttitudeRate cmd;

        cmd.throttle = msg->throttle;
        cmd.rate_x = msg->body_rate.x;
        cmd.rate_y = msg->body_rate.y;
        cmd.rate_z = msg->body_rate.z;

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::ATTITUDE_RATE_CMD;
        }
    }

    //}

    /* callbackAttitudeCmd() //{ */

    void UavSystemRos::callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting attitude command", _uav_name_.c_str());

        reference::Attitude cmd;

        cmd.throttle = msg->throttle;

        cmd.orientation = mrs_lib::AttitudeConverter(msg->orientation);

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::ATTITUDE_CMD;
        }
    }

    //}

    /* callbackAccelerationHdgRateCmd() //{ */

    void UavSystemRos::callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting acceleration+hdg rate command", _uav_name_.c_str());

        reference::AccelerationHdgRate cmd;

        cmd.heading_rate = msg->heading_rate;

        cmd.acceleration(0) = msg->acceleration.x;
        cmd.acceleration(1) = msg->acceleration.y;
        cmd.acceleration(2) = msg->acceleration.z;

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::ACCELERATION_HDG_RATE_CMD;
        }
    }

    //}

    /* callbackAccelerationHdgCmd() //{ */

    void UavSystemRos::callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting acceleration+hdg command", _uav_name_.c_str());

        reference::AccelerationHdg cmd;

        cmd.heading = msg->heading;

        cmd.acceleration(0) = msg->acceleration.x;
        cmd.acceleration(1) = msg->acceleration.y;
        cmd.acceleration(2) = msg->acceleration.z;

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::ACCELERATION_HDG_CMD;
        }
    }

    //}

    /* callbackVelocityHdgRateCmd() //{ */

    void UavSystemRos::callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting velocity+hdg rate command", _uav_name_.c_str());

        reference::VelocityHdgRate cmd;

        cmd.heading_rate = msg->heading_rate;

        cmd.velocity(0) = msg->velocity.x;
        cmd.velocity(1) = msg->velocity.y;
        cmd.velocity(2) = msg->velocity.z;

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::VELOCITY_HDG_RATE_CMD;
        }
    }

    //}

    /* callbackVelocityHdgCmd() //{ */

    void UavSystemRos::callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting velocity+hdg command", _uav_name_.c_str());

        reference::VelocityHdg cmd;

        cmd.heading = msg->heading;

        cmd.velocity(0) = msg->velocity.x;
        cmd.velocity(1) = msg->velocity.y;
        cmd.velocity(2) = msg->velocity.z;

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::VELOCITY_HDG_CMD;
        }
    }

    //}

    /* callbackPositionCmd() //{ */

    void UavSystemRos::callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting position command", _uav_name_.c_str());

        reference::Position cmd;

        cmd.heading = msg->heading;

        cmd.position(0) = msg->position.x;
        cmd.position(1) = msg->position.y;
        cmd.position(2) = msg->position.z;

        {
            std::scoped_lock lock(mutex_uav_system_);

            uav_system_.setInput(cmd);
        }

        {
            std::scoped_lock lock(mutex_time_last_input_);

            time_last_input_ = ros::Time::now();
            last_input_mode_ = UavSystem::POSITION_CMD;
        }
    }

    //}

    /* callbackTrackerCmd() //{ */

    void UavSystemRos::callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        ROS_INFO_ONCE("[%s]: getting tracker command", _uav_name_.c_str());

        Eigen::Vector3d velocity(0, 0, 0);
        Eigen::Vector3d acceleration(0, 0, 0);
        double heading_rate = 0;

        if (msg->use_velocity_horizontal)
        {
            velocity(0) = msg->velocity.x;
            velocity(1) = msg->velocity.y;
        }

        if (msg->use_velocity_vertical)
        {
            velocity(2) = msg->velocity.z;
        }

        if (msg->use_heading_rate)
        {
            heading_rate = msg->heading_rate;
        }

        if (msg->use_acceleration)
        {
            acceleration(0) = msg->acceleration.x;
            acceleration(1) = msg->acceleration.y;
            acceleration(2) = msg->acceleration.z;
        }

        uav_system_.setFeedforward(reference::VelocityHdg(velocity, 0));
        uav_system_.setFeedforward(reference::VelocityHdgRate(velocity, heading_rate));
        uav_system_.setFeedforward(reference::AccelerationHdg(acceleration, 0));
        uav_system_.setFeedforward(reference::AccelerationHdgRate(acceleration, heading_rate));
    }

    //}

    /* callbackSetMass() //{ */

    bool UavSystemRos::callbackSetMass(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res)
    {

        if (!is_initialized_)
        {
            return false;
        }

        {
            std::scoped_lock lock(mutex_uav_system_);

            model_params_ = uav_system_.getParams();

            const double original_mass = model_params_.mass;

            model_params_.mass = req.value;

            model_params_.allocation_matrix.row(2) = model_params_.mass * (model_params_.allocation_matrix.row(2) / original_mass);

            calculateInertia(model_params_);

            uav_system_.setParams(model_params_);
        }

        res.success = true;
        res.message = "mass set";

        return true;
    }

    //}

    /* callbackSetGroundZ() //{ */

    bool UavSystemRos::callbackSetGroundZ(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res)
    {

        if (!is_initialized_)
        {
            return false;
        }

        {
            std::scoped_lock lock(mutex_uav_system_);

            model_params_ = uav_system_.getParams();

            model_params_.ground_z = req.value;

            uav_system_.setParams(model_params_);
        }

        res.success = true;
        res.message = "ground z set";

        return true;
    }

    //}

} // namespace mrs_multirotor_simulator
