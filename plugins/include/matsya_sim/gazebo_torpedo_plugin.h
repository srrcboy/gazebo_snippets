/* \file    gazebo_torpedo_plugin.h
 * \license BSD
 * \author  Kunal Tyagi "tyagi.kunal@live.com"
 * \version 0.5
 * \date    12 Oct 2013
 * \about   Header file for gazebo_torpedo_plugin.cpp
 * \desc    Gazebo plugin which will interact with the Matsya simulation
 *          and will control the torpedoes
 */

#ifndef GAZEBO_TORPEDO_PLUGIN_HH
#define GAZEBO_TORPEDO_PLUGIN_HH

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

//rosmsgs includes
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <auv_msgs/shootTorpedo.h>

//custom includes

//macros
#define TORPEDO_LEFT_FORCE_UPDATES 50.0
#define TORPEDO_RIGHT_FORCE_UPDATES 50.0
#define TORPEDO_FORCE 0.01
#define TORPEDO_G 1.0
#define TORPEDO_LINEAR_DRAG 0.001
#define TORPEDO_ANGULAR_DRAG 0.001
#define TORPEDO_MASS 0.001
#define NO_OF_MODELS 5

namespace gazebo
{
    class TorpedoPlugin : public ModelPlugin
    {
      public:
        /// \brief Constructor
        TorpedoPlugin();

        /// \brief Destructor
        ~TorpedoPlugin();

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

        /// \brief Links for torpedoes
        physics::LinkPtr _torpedoLeftLink;
        physics::LinkPtr _torpedoRightLink;
        bool _torpedoLeftFired;
        bool _torpedoRightFired;

        /// \brief pointer to ros node
        ros::NodeHandle *_nh;
        ros::ServiceServer _fireTorpedoServer;

        /// \brief Carry the Torpedoes
        void carryTorpedoes();

        /// \brief Shoot on request
        bool shootTorpedoCallback(auv_msgs::shootTorpedo::Request& req, auv_msgs::shootTorpedo::Response &res);

        /// \brief check for ROS updates
        void spin();

        /// \brief deferred load in case ros is blocking
        boost::thread _spinnerThread;
        void loadThread();
        sdf::ElementPtr _sdf;

        /// \brief Pointer to update event connection
        event::ConnectionPtr _updateConnection;

        /// \brief ros messages and services

        /// \brief Save the torpedo count
        int _leftTorpedoCount;
        int _rightTorpedoCount;
    };

} //EON
#endif
