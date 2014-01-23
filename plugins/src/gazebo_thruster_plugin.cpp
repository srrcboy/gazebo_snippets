/* \file    gazebo_thruster_plugin.cpp
 * \author  Kunal Tyagi "tyagi.kunal@live.com"
 * \license BSD
 * \version 1.0
 * \date    11 Oct 2013
 * \about   Functions for header file for gazebo_thruster_plugin.h
 * \desc    Contains function definitions for the functions responsible
 *          for making the plugin run
 */

//header include
#include <matsya_sim/gazebo_thruster_plugin.h>

namespace gazebo
{
/// \brief Register the plugin
GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin);

//Constructor
ThrusterPlugin::ThrusterPlugin()
{
//    this->_spinnerThread = new boost::thread( boost::bind( &ThrusterPlugin::spin, this) );
}

//Destructor
ThrusterPlugin::~ThrusterPlugin()
{
    event::Events::DisconnectWorldUpdateBegin(this->_updateConnection);
    // Finalize the controller
    this->_nh->shutdown();
    this->_spinnerThread.join();
    delete this->_nh;
}

//Load the controller
void ThrusterPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // save pointers
    this->_matsyaWorld = _parent->GetWorld();
    this->_matsyaModel = _parent;
    this->_baseLink  = _matsyaModel->GetLink("base_link");
    this->_sdf = _sdf;
    //double mass = _baseLink->GetInertial()->GetMass();
    //gzdbg << "base mass is: "<< mass <<"\n";

    // ros callback queue for processing subscription
    this->_spinnerThread = boost::thread(
            boost::bind(&ThrusterPlugin::loadThread, this));
}

//Load the controller
void ThrusterPlugin::loadThread()
{
    //load parameters
    this->_namespace = "";
    if(this->_sdf->HasElement("robotNamespace"))
        this->_namespace = this->_sdf->Get<std::string>("robotNamespace")
            + "/";

    if(!this->_matsyaModel){
        ROS_FATAL_STREAM("Gazebo Thruster controller requires a model as its parent");
        return;
    }

    //std::string modelName = _sdf->GetParent()->GetValueString("name");

    //gzdbg << "plugin model name: "<<modelName << "\n";

    // Make sure the ROS node for Gazebo has already been initialized
    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for gazebo has not been initialized"
            <<", unable to load plugin: ThrusterPlugin. "
            <<"Load Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
            <<"in the gazebo_ros package");
        return;
    }

    _nh = new ros::NodeHandle(this->_namespace);

    _pwmValueSub = _nh->subscribe("/sim/motor_pwm_values", 1,
            &ThrusterPlugin::pwmValuesCallBack, this);
    //_posePub = _nh->advertise<auv_msgs::Pose>("/sim/world_pose",1);
    _pwm.values.clear();
    //Initialize all thrusters to zero speed.
    for (int i=0; i<NO_OF_THRUSTERS; ++i){
        _pwm.values.push_back(THRUSTER_STOP_PWM);
    }

    _dropMarkerServer = _nh->advertiseService("/sim/marker_drop",
            &ThrusterPlugin::dropMarkerCallback, this);

    _markerLeftDropped = false;
    _markerRightDropped = false;

    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ThrusterPlugin::updateChild, this));
    this->spin();
}


/// \brief:  At each update step, change force according to the PWM values
///          Model: The force depends on the angular velocity
///                 of the thruster, which itself is proportional
///                 to the dc of PWM pulse supplied to the motor
///          Actual: thrust varies as square of the pwm (angular speed)

// Update the controller
void ThrusterPlugin::updateChild()
{
    math::Vector3 force(0.0,0.0,0.0);
    math::Vector3 torque(0.0,0.0,0.0);
    _baseLink->SetTorque(torque);
    _baseLink->SetForce(force);
    applyThrusterForce();
    applyDragFriction();
}

// Move markers with Matsya
//void ThrusterPlugin::carryMarkers()
//{
    //@TODO No markers yet!
//}

// Move Matsya
void ThrusterPlugin::applyThrusterForce()
{
    math::Vector3 force(0.0,0.0,0.0);
    math::Vector3 torque(0.0,0.0,0.0);
    math::Quaternion poseRot = _baseLink->GetWorldPose().rot;
    for(int i=0; i<NO_OF_THRUSTERS; ++i)
    {
        // if(_pwm.values[i] > 1023)
        //     _pwm.values[i] = 1023;
        // else if(_pwm.values[i] < 0)
        //     _pwm.values[i] = 0;

        switch(i)
        {
            case THRUSTER_SURGE_LEFT :
                force.x += (_pwm.values[THRUSTER_SURGE_LEFT]
                        -THRUSTER_STOP_PWM)/FORCE_PWM_FACTOR;
                torque.z += -(_pwm.values[THRUSTER_SURGE_LEFT]
                        -THRUSTER_STOP_PWM)/TORQUE_PWM_FACTOR;
                break;
            case THRUSTER_SURGE_RIGHT :
                force.x += (_pwm.values[THRUSTER_SURGE_RIGHT]
                        -THRUSTER_STOP_PWM)/FORCE_PWM_FACTOR;
                torque.z += (_pwm.values[THRUSTER_SURGE_RIGHT]
                        -THRUSTER_STOP_PWM)/TORQUE_PWM_FACTOR;
                break;
            case THRUSTER_SWAY_BACK :
                force.y += -(_pwm.values[THRUSTER_SWAY_BACK]
                        -THRUSTER_STOP_PWM)/FORCE_PWM_FACTOR;
                torque.z += -(_pwm.values[THRUSTER_SWAY_BACK]
                        -THRUSTER_STOP_PWM)/TORQUE_PWM_FACTOR;
                break;
            case THRUSTER_SWAY_FRONT :
                force.y += (_pwm.values[THRUSTER_SWAY_FRONT]
                        -THRUSTER_STOP_PWM)/FORCE_PWM_FACTOR;
                torque.z += -(_pwm.values[THRUSTER_SWAY_FRONT]
                        -THRUSTER_STOP_PWM)/TORQUE_PWM_FACTOR;
                break;
            case THRUSTER_DEPTH_BACK :
                force.z += -(_pwm.values[THRUSTER_DEPTH_BACK]
                        -THRUSTER_STOP_PWM)/FORCE_PWM_FACTOR;
                torque.y += -(_pwm.values[THRUSTER_DEPTH_BACK]
                        -THRUSTER_STOP_PWM)/TORQUE_PWM_FACTOR;
                break;
            case THRUSTER_DEPTH_FRONT :
                force.z += -(_pwm.values[THRUSTER_DEPTH_FRONT]
                        -THRUSTER_STOP_PWM)/FORCE_PWM_FACTOR;
                torque.y += (_pwm.values[THRUSTER_DEPTH_FRONT]
                        -THRUSTER_STOP_PWM)/TORQUE_PWM_FACTOR;
                break;
            default :
                break;
        }
    }
    torque.x = 0;
    force = poseRot.RotateVector(force);
    torque = poseRot.RotateVector(torque);
    // gzdbg<<force<<'\t'<<torque<<'\t';
    _baseLink->AddRelativeTorque(torque);
    _baseLink->AddRelativeForce(force);
    // gzdbg<<_baseLink->GetWorldPose()<<'\n';
}

// Apply friction
void ThrusterPlugin::applyDragFriction()
{
    math::Vector3 _lin_vel = _baseLink->GetWorldLinearVel();
    math::Vector3 _ang_vel = _baseLink->GetWorldAngularVel();
    double lin_factor = -LINEAR_DRAG_FACTOR;
    double ang_factor = -ANGULAR_DRAG_FACTOR;
    math::Vector3 drag_force = _lin_vel / lin_factor;
    math::Vector3 drag_torque = _ang_vel / ang_factor;
    //std::cout << _lin_vel << " a " << _ang_vel << std::endl;
    _baseLink->AddRelativeForce(drag_force);
    _baseLink->AddRelativeTorque(drag_torque);
}

// Update PWM values
void ThrusterPlugin::pwmValuesCallBack(const auv_msgs::PWMValues pwm)
{
    this->_pwm.values = pwm.values;
}

// Get orders to drop Markers
bool ThrusterPlugin::dropMarkerCallback(auv_msgs::dropMarker::Request& req, auv_msgs::dropMarker::Response& res)
{
    return false;
}

//Publish the simulated pose
void ThrusterPlugin::publishPose()
{
    // _depth.data = ((float) _surfaceZ - d)*100;
    // _ps.adc_data = (uint16_t) (_depth.data * PRESSURE_SLOPE +
    //         PRESSURE_INERCEPT);
    // _depthPub.publish(_ps);
}

// Check for new messages
void ThrusterPlugin::spin()
{
    while(ros::ok()){
        ros::spinOnce();
    }
}

} //EON
