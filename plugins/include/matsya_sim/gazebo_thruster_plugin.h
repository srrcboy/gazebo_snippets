/* \file    gazebo_thruster_plugin.h
 * \license BSD
 * \author  Kunal Tyagi "tyagi.kunal@live.com"
 * \version 1.0
 * \date    11 Oct 2013
 * \about   Header file for gazebo_thruster_plugin.cpp
 * \desc    Gazebo plugin which will interact with the Matsya simulation
 *          and act as a pressure sensor publishing the thruster of the
 *          vehicle
 */

#ifndef GAZEBO_THRUSTER_PLUGIN_HH
#define GAZEBO_THRUSTER_PLUGIN_HH

//includes
#include <string>

//ros includes
#include <ros/ros.h>

//boost includes
#include <boost/thread/thread.hpp>

//gazebo includes
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/CommonTypes.hh>

//rosmsgs includes
#include <sensor_msgs/JointState.h>
#include <auv_msgs/PWMValues.h>
#include <auv_msgs/dropMarker.h>
#include <auv_msgs/Pose.h>

//custom includes

//macros
//Thrusters
#define THRUSTER_SWAY_BACK 0
#define THRUSTER_SURGE_LEFT 1
#define THRUSTER_DEPTH_BACK 2
#define THRUSTER_DEPTH_FRONT 3
#define THRUSTER_SURGE_RIGHT 4
#define THRUSTER_SWAY_FRONT 5
//Regular
#define NO_OF_THRUSTERS 6.0
#define FORCE_PWM_FACTOR 512.0
#define THRUSTER_STOP_PWM 512.0
#define TORQUE_PWM_FACTOR 5120.0
#define ANGULAR_DRAG_FACTOR 5.0
#define LINEAR_DRAG_FACTOR 0.1 //decrease for increasing drag

namespace gazebo
{
    class ThrusterPlugin : public ModelPlugin
    {
      public:
        /// \brief Constructor
        ThrusterPlugin();

        /// \brief Destructor
        ~ThrusterPlugin();

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

        /// \brief to set ROS namespace
        std::string _namespace;

        /// \brief Links for markers
        physics::LinkPtr _markerLeftLink;
        physics::LinkPtr _markerRightLink;
        bool _markerLeftDropped;
        bool _markerRightDropped;

        /// \brief pointer to ros node
        ros::NodeHandle *_nh;
        ros::Subscriber _pwmValueSub;
        ros::Publisher _posePub;
        ros::ServiceServer _fireTorpedoServer;
        ros::ServiceServer _dropMarkerServer;

        /// \brief ros messages and services
        auv_msgs::PWMValues _pwm;

        /// \brief deferred load in case ros is blocking
        boost::thread _spinnerThread;
        void loadThread();
        sdf::ElementPtr _sdf;

        /// \brief Pointer to update event connection
        event::ConnectionPtr _updateConnection;

        /// \brief check for ROS updates
        void spin();

        /// \brief Publish the pose of the vehicle
        void publishPose();

        /// \brief Receive PWM values, orders to shoot/drop
        void pwmValuesCallBack(const auv_msgs::PWMValues pwm);
        bool dropMarkerCallback(auv_msgs::dropMarker::Request& req, auv_msgs::dropMarker::Response& res);

        /// \brief Apply the changes in Gazebo environment
        void applyThrusterForce();
        void applyDragFriction();

    };

} //EON

#endif
