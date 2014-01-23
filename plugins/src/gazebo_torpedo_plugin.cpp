/* \file    gazebo_torpedo_plugin.cpp
 * \author  Kunal Tyagi "tyagi.kunal@live.com"
 * \license BSD
 * \version 0.5
 * \date    12 Oct 2013
 * \about   Function for header file for gazebo_torpedo_plugin.h
 * \desc    Contains function definitions for the functions responsible
 *          for making the plugin run
 */

//header include
#include <matsya_sim/gazebo_torpedo_plugin.h>

namespace gazebo
{
/// \brief Register the plugin
GZ_REGISTER_MODEL_PLUGIN(TorpedoPlugin);

//Constructor
TorpedoPlugin::TorpedoPlugin()
{
	;
}

//Destructor
TorpedoPlugin::~TorpedoPlugin()
{
    event::Events::DisconnectWorldUpdateBegin(this->_updateConnection);
    // Finalize the controller
    this->_nh->shutdown();
    this->_spinnerThread.join();
    delete this->_nh;
}

//Load the controller
void TorpedoPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // save pointers
    this->_matsyaWorld = _parent->GetWorld();
    this->_matsyaModel = _parent;
    this->_baseLink  = _matsyaModel->GetLink("base_link");
    this->_sdf = _sdf;

    // ros callback queue for processing subscription
    this->_spinnerThread = boost::thread(
            boost::bind(&TorpedoPlugin::loadThread, this));
}

//Load the controller
void TorpedoPlugin::loadThread()
{
    // load parameters
    this->_namespace = "";
    if(this->_sdf->HasElement("robotNamespace"))
        this->_namespace = this->_sdf->Get<std::string>("robotNamespace")
            + "/";

    if(!this->_matsyaModel){
        ROS_FATAL_STREAM("Gazebo Torpedo Plugin requires a model as its parent");
        return;
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for gazebo has not been intialized"
            <<", unable to load plugin: TorpedoPlugin. "
            <<"Load Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
            <<"in the gazebo_ros package");
        return;
    }

    _nh = new ros::NodeHandle(this->_namespace);

    _fireTorpedoServer = _nh->advertiseService("/sim/torpedo_shoot",
            &TorpedoPlugin::shootTorpedoCallback, this);

    _torpedoLeftFired = false;
    _torpedoRightFired = false;
    _leftTorpedoCount = 0;
    _rightTorpedoCount = 0;

    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&TorpedoPlugin::updateChild, this));
    this->spin();
}

// Update the controller
void TorpedoPlugin::updateChild()
{
    math::Vector3 force(0.0,0.0,0.0);
    math::Vector3 torque(0.0,0.0,0.0);
    if (_matsyaWorld->GetModelCount() == NO_OF_MODELS)
    {
    	//Wait for torpedoes to spawn?
        //gzdbg << _matsyaWorld->GetModelCount() << "\n";
        this->_torpedoLeftLink = _matsyaWorld->GetModel(
                "torpedo_left")->GetLink("_baseLink");
        this->_torpedoRightLink = _matsyaWorld->GetModel(
                "torpedo_right")->GetLink("_baseLink");
        carryTorpedoes();
    }
}

// Move Torpedoes with Matsya
void TorpedoPlugin::carryTorpedoes()
{
    // Initialize all the forces
    math::Vector3 force(0.0,0.0,0.0);
    math::Vector3 torque(0.0,0.0,0.0);
    _torpedoLeftLink->SetTorque(torque);
    _torpedoLeftLink->SetForce(force);
    _torpedoRightLink->SetForce(force);
    _torpedoRightLink->SetTorque(torque);

    math::Vector3 horizontal_force(TORPEDO_FORCE,0,0);
    math::Vector3 vertical_force(0,0,-TORPEDO_MASS*TORPEDO_G);
    math::Quaternion poseRot = _baseLink->GetWorldPose().rot;

    if (!_torpedoLeftFired)
    {
        _torpedoLeftLink->SetLinearVel(_baseLink->GetWorldLinearVel());
        _torpedoLeftLink->SetAngularVel(_baseLink->GetWorldAngularVel());
    } else
    {
        //Also apply downward force normally there on  bodies,
        //something less than the gravitational force.
        if (_leftTorpedoCount < TORPEDO_LEFT_FORCE_UPDATES)
        {
            _leftTorpedoCount++;
            math::Vector3 new_horizontal_force = poseRot.RotateVector(
                    horizontal_force);
            _torpedoLeftLink->AddRelativeForce(new_horizontal_force);
        }

        math::Vector3 new_vertical_force = poseRot.RotateVector(
                vertical_force);
        _torpedoLeftLink->AddRelativeForce(new_vertical_force);

        // Drag friction on torpedo
        math::Vector3 _lin_vel = _torpedoLeftLink->GetWorldLinearVel();
        math::Vector3 _ang_vel = _torpedoLeftLink->GetWorldAngularVel();

        //@TODO
        //Drag force is proportional to v^2 at high v!
        //check value of _lin_vel and the code
        math::Vector3 drag_force = _lin_vel * -TORPEDO_LINEAR_DRAG;
        math::Vector3 drag_torque = _ang_vel * -TORPEDO_ANGULAR_DRAG;

        _torpedoLeftLink->AddRelativeForce(drag_force);
        _torpedoLeftLink->AddRelativeTorque(drag_torque);
    }

    //Copy of leftTorpedoCode
    if (!_torpedoRightFired)
    {
        _torpedoRightLink->SetLinearVel(_baseLink->GetWorldLinearVel());
        _torpedoRightLink->SetAngularVel(_baseLink->GetWorldAngularVel());
    } else
    {
        if (_rightTorpedoCount < TORPEDO_RIGHT_FORCE_UPDATES)
        {
            _rightTorpedoCount++;
            math::Vector3 new_horizontal_force = poseRot.RotateVector(
                    horizontal_force);
            _torpedoRightLink->AddRelativeForce(new_horizontal_force);
        }
        math::Vector3 new_vertical_force = poseRot.RotateVector(
                vertical_force);
        _torpedoRightLink->AddRelativeForce(new_vertical_force);

        // Drag Friction on the torpedo
        math::Vector3 _lin_vel = _torpedoRightLink->GetWorldLinearVel();
        math::Vector3 _ang_vel = _torpedoRightLink->GetWorldAngularVel();

        math::Vector3 drag_force = _lin_vel * -TORPEDO_LINEAR_DRAG;
        math::Vector3 drag_torque = _ang_vel * -TORPEDO_ANGULAR_DRAG;

        _torpedoRightLink->AddRelativeForce(drag_force);
        _torpedoRightLink->AddRelativeTorque(drag_torque);
    }
}

// Get orders to shoot Torpedoes
bool TorpedoPlugin::shootTorpedoCallback(auv_msgs::shootTorpedo::Request& req, auv_msgs::shootTorpedo::Response& res)
{
    if (req.torpedo == auv_msgs::shootTorpedo::Request::TORPEDO_LEFT)
    {
        if (!_torpedoLeftFired)
        {
            _torpedoLeftFired = true;
            res.status = true;
        } else
        {
            res.status = false;
        }
    } else
    if (req.torpedo == auv_msgs::shootTorpedo::Request::TORPEDO_RIGHT)
    {
        if (!_torpedoRightFired)
        {
            _torpedoRightFired = true;
            res.status = true;
        } else
        {
            res.status = false;
        }
    }
    return true;
}

// Check for new messages
void TorpedoPlugin::spin()
{
    while(ros::ok()){
        ros::spinOnce();
    }
}

} //EON
