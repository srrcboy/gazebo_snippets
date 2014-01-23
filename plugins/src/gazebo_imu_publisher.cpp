/* \file    gazebo_imu_plugin.cpp
 * \author  Kunal Tyagi "tyagi.kunal@live.com"
 * \license BSD
 * \version 1.0
 * \date    4 Oct 2013
 * \version 0.75
 * \date    19 Oct 2013
 * \about   Functions for header file gazebo_imu_plugin.h
 * \desc    Contains function definitions for the functions responsible
 *          for making the plugin run
 */

#include <matsya_sim/gazebo_imu_publisher.h>
namespace gazebo
{
static int counter = 0;
ros::NodeHandle _nh;
ros::Publisher _imuPub;

/// \brief Publish the IMU readings on a ROS topic
void parseInput(ConstIMUPtr &_msg)
{
    // Dump the message contents to stdout.
    std::stringstream readings;
    readings << _msg->DebugString();

    Header header;
    time stamp;
    Quaternion orientation;
    Vector3 angular_velocity, linear_acceleration;

    std::string tmp, entity_name, frame_id="1";
/*
    format:
stamp {
  sec: 70
  nsec: 254000000
}
entity_name: "Matsya::base_link"
orientation {
  x: 0
  y: 0
  z: 0
  w: 1
}
angular_velocity {
  x: -0.00010220566589886817
  y: 1.7110388106038058e-05
  z: 0.0001448255917663688
}
linear_acceleration {
  x: -0.094560571804233481
  y: -0.12642955252116528
  z: -0.11442258526591488
}
*/
    //stamp
    readings>>tmp>>tmp>>tmp>>stamp.sec>>tmp>>stamp.nsec>>tmp;
    //entity name
    readings>>tmp>>entity_name;
    //orientation
    readings>>tmp>>tmp>>tmp>>orientation.x>>tmp>>orientation.y>>tmp>>orientation.z>>tmp>>orientation.w>>tmp;
    //angular_velocity
    readings>>tmp>>tmp>>tmp>>angular_velocity.x>>tmp>>angular_velocity.y>>tmp>>angular_velocity.z>>tmp;
    //linear_acceleration
    readings>>tmp>>tmp>>tmp>>linear_acceleration.x>>tmp>>linear_acceleration.y>>tmp>>linear_acceleration.z>>tmp;
    //readings is now empty

    sensor_msgs::Imu currentmsg;
    currentmsg.header.seq = counter;
    currentmsg.header.stamp.sec = stamp.sec;
    currentmsg.header.stamp.nsec = stamp.nsec;
    currentmsg.header.frame_id = frame_id;

    for(int i=0; i<9; i++) //disable co variance
    {
        currentmsg.orientation_covariance[i] = currentmsg.angular_velocity_covariance[i] = currentmsg.linear_acceleration_covariance[i] = -1;
    }

    currentmsg.orientation.x = orientation.x;
    currentmsg.orientation.y = orientation.y;
    currentmsg.orientation.z = orientation.z;
    currentmsg.orientation.w = orientation.w;

    currentmsg.angular_velocity.x = angular_velocity.x;
    currentmsg.angular_velocity.y = angular_velocity.y;
    currentmsg.angular_velocity.z = angular_velocity.z;

    currentmsg.linear_acceleration.x = linear_acceleration.x;
    currentmsg.linear_acceleration.y = linear_acceleration.y;
    currentmsg.linear_acceleration.z = linear_acceleration.z;

    _imuPub.publish(currentmsg);

    counter++;
}

int main(int _argc, char **_argv)
{
ros::Publisher _imuPub =  _nh.advertise<sensor_msgs::Imu>("/sim/imu/imu_raw",1);

    // Load gazebo
    gazebo::load(_argc, _argv);

    gazebo::run();

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();


    // Listen to Gazebo imu topic
    gazebo::transport::SubscriberPtr sub =  node->Subscribe("/sim/imu/imu_raw", parseInput);

    // Busy wait loop...replace with your own code as needed.
    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    gazebo::transport::fini();
}
}
