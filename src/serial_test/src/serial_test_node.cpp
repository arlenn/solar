#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <string>

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "serial_test_node");  //we can change or remap this

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher serial_test_str_pub = nh.advertise<std_msgs::String>("hello_world", 1);
  ros::Publisher serial_test_bool_pub = nh.advertise<std_msgs::Bool>("led_on", 1);
  ros::Rate loop_rate(1);  // Hz
  
  while ( ros::ok() )
  { 
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */    
    static int i = 0;
    std::ostringstream ss; 
    ss << "hello, micro (" << i << ")";
    std::string serial_str = ss.str();
    std_msgs::String serial_str_pub;
    serial_str_pub.data = serial_str;

    static bool serial_bool = false;
    std_msgs::Bool serial_bool_pub; 
    serial_bool_pub.data = serial_bool;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    serial_test_str_pub.publish(serial_str_pub);
    ROS_INFO_STREAM("ROS sent: " << serial_str);
    i++;

    serial_test_bool_pub.publish(serial_bool_pub);
    ROS_INFO_STREAM("ROS sent: " << serial_bool);
    serial_bool = !serial_bool; //maybe this will flash the led?

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}