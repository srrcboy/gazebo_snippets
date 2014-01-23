/* \file    gazebo_depth_plugin.cpp
 * \author  Kunal Tyagi "tyagi.kunal@live.com"
 * \license BSD
 * \version 1.0
 * \date    4 Oct 2013
 * \about   Functions for header file gazebo_depth_plugin.h
 * \desc    Contains function definitions for the functions responsible
 *          for making the plugin run
 */

//header include
#include <matsya_sim/gazebo_depth_plugin.h>

namespace gazebo
{
/// \brief Register the plugin
GZ_REGISTER_MODEL_PLUGIN(DepthPlugin);

//Constructor
DepthPlugin::DepthPlugin()
{
//    this->_spinnerThread = new boost::thread( boost::bind( &DepthPlugin::spin, this) );
}

//Destructor
DepthPlugin::~DepthPlugin()
{
    event::Events::DisconnectWorldUpdateBegin(this->_updateConnection);
    // Finalize the controller
    this->_nh->shutdown();
    this->_spinnerThread.join();
//  delete this->_spinnerThread;
    delete this->_nh;
}

//Load the controller
void DepthPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // save pointers
    this->_matsyaWorld = _parent->GetWorld();
    this->_matsyaModel = _parent;
    this->_baseLink  = _matsyaModel->GetLink("base_link");
    this->_sdf = _sdf;

    // ros callback queue for processing subscription
    this->_spinnerThread = boost::thread(
            boost::bind(&DepthPlugin::loadThread, this));
}

//Load the controller
void DepthPlugin::loadThread()
{
    // load parameters
    this->_namespace = "";
    if(this->_sdf->HasElement("robotNamespace"))
        this->_namespace = this->_sdf->Get<std::string>("robotNamespace")
            + "/";

    if(!this->_matsyaModel){
        ROS_FATAL_STREAM("Gazebo Depth Plugin requires a model as its parent");
        return;
    }

    //std::string modelName = _sdf->GetParent()->GetValueString("name");
    //gzdbg << "plugin model name: "<<modelName << "\n";

    // Make sure the ROS node for Gazebo has already been initialized
    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for gazebo has not been intialized"
            <<", unable to load plugin: DepthPlugin. "
            <<"Load Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
            <<"in the gazebo_ros package");
        return;
        /*
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "depth_publish_node", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        */
    }

    _nh = new ros::NodeHandle(this->_namespace);

    _depthPub = _nh->advertise<auv_msgs::PressureSensor>("/sim/pressure_sensor_data",1);
    _surfaceZ = SURFACE_Z;
    _updateParam = _nh->advertiseService("/simulator/update_parameters",
            &DepthPlugin::updateParameters, this);

    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DepthPlugin::updateChild, this));

    this->spin();
}

// Update the controller
void DepthPlugin::updateChild()
{
    publishDepth();
}

// Publish the simulated data
void DepthPlugin::publishDepth()
{
    //Get the z coordinate of models absolute pose
    float d = _matsyaModel->GetWorldPose().pos.z;
    //float surface = 10; //@ TODO : make it a param
    //gzdbg << "depth is: " << d << "\n"; //for debug
    _depth.data = ((float) _surfaceZ - d)*100;
    _ps.adc_data = (uint16_t) (_depth.data * PRESSURE_SLOPE +
            PRESSURE_INERCEPT);
    _depthPub.publish(_ps);
}

// Check for new messages
void DepthPlugin::spin()
{
    while(ros::ok()){
        ros::spinOnce();
    }
}

// Update parameters from the server
bool DepthPlugin::updateParameters(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    _nh->getParam("/sim/water_surface",_surfaceZ);
    return true;
}

} //EON
