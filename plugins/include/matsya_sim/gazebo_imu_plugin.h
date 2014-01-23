/* \file    gazebo_imu_plugin.h
 * \license BSD
 * \author  Kunal Tyagi "tyagi.kunal@live.com"
 * \version 1.0
 * \date    4 Oct 2013
 * \about   Header file for gazebo_imu_plugin.cpp
 * \desc    Gazebo plugin which will publish IMU data on ROS topic
 */

#ifndef GAZEBO_IMU_PLUGIN_HH
#define GAZEBO_IMU_PLUGIN_HH

//includes
#include <string>

//ros includes
#include <ros/ros.h>

//boost includes
#include <boost/thread/thread.hpp>

//gazebo includes
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

//rosmsgs includes
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <auv_msgs/Pose.h>

//custom includes

//macros
#define SURFACE_Z  5
#define PRESSURE_INERCEPT 200
#define PRESSURE_SLOPE 6

namespace gazebo
{
    class ImuPlugin : public ModelPlugin
    {
      public:
        /// \brief Constructor
        ImuPlugin();

        /// \brief Destructor
        ~ImuPlugin();

        /// \brief Load the controller
        virtual void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf);

      protected:
        /// \brief Update the controller
        virtual void updateChild();

      private:
        /// \brief The Matsya Model
        physics::ModelPtr _matsyaModel;

        /// \brief The parent World
        physics::WorldPtr _matsyaWorld;

        /// \brief The Link referred to by this plugin
        physics::LinkPtr _baseLink;

        /// \brief To set ROS namespace
        std::string _namespace;

        /// \brief The sensor
        ImuSensor _imu;

        /// \brief pointer to ros node
        ros::NodeHandle *_nh;
        ros::Publisher _imuPub;

        /// \brief Publish Pressure Sensor data
        void publishImu();

        /// \brief Update the imu from surface
        bool updateParameters(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        /// \brief check for ROS updates
        void spin();

        /// \brief deferred load in case ros is blocking
        boost::thread _spinnerThread;
        void loadThread();
        sdf::ElementPtr _sdf;

        /// \brief Pointer to update event connection
        event::ConnectionPtr _updateConnection;

        /// \brief ros messages and services

    };

} //EON
#endif
