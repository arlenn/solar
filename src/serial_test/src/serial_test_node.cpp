#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <string>

class ledPublish
{ 
  public:
    ledPublish(ros::NodeHandle *nh) :
      // initialize
      m_serial_bool(0) 
    {
      m_serial_test_bool_pub = nh->advertise<std_msgs::Bool>("led_on", 1);
    }

    void publishBool(const ros::TimerEvent& event)
    { 
      m_serial_bool_pub.data = m_serial_bool;
      m_serial_test_bool_pub.publish(m_serial_bool_pub);
      ROS_INFO_STREAM("ROS sent: " << m_serial_bool);
      m_serial_bool = !m_serial_bool;
    }

  private:
    bool m_serial_bool;
    std_msgs::Bool m_serial_bool_pub;
    ros::Publisher m_serial_test_bool_pub;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_test_node");  //we can change or remap this
  ros::NodeHandle nh;

  ledPublish ledPub(&nh);
  
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), &ledPublish::publishBool, &ledPub);

  // ros::Rate rate(1); // Hz

  while ( ros::ok() )
  {
    // ledPub.publishBool();

    // ros::spinOnce();
    // rate.sleep();
    ros::spin();
  }
  return 0;
}
