#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float32.h"  // We will have to check on the MCUs capability

#include <ctime>
#include <cstdlib>

extern "C" // Required for C to be useable within C++
{
  #include "SolTrack.h" //solar tracking library
}


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
  ros::init(argc, argv, "solar_track_node");  //we can change or remap this

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
  ros::Publisher solar_position_pub = nh.advertise<std_msgs::Float32>("solar_position", 1);

  ros::Rate loop_rate(1);  // Hz

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while ( ros::ok() )
  {
    // TODO According to ROS docs. this might need to be gaurded against a 0 value!
    std::time_t epochNow = ros::Time::now().toSec();

    std::tm localTimeNow = *localtime(&epochNow);
    std::tm utcTimeNow = *gmtime(&epochNow);

    // DEBUG: Print various components of tm structure.
    
    ROS_INFO_STREAM("Local Time/date as reported by system: "
      << 1900 + localTimeNow.tm_year << "-" // Years since 1900
      << 1 + localTimeNow.tm_mon << "-"  // A number of tm fields are 0 indexed
      << localTimeNow.tm_mday << ","
      << 1 + localTimeNow.tm_hour << ":"
      << 1 + localTimeNow.tm_min << ":"
      << 1 + localTimeNow.tm_sec << ","
      << "DST:" << localTimeNow.tm_isdst << "," 
      << localTimeNow.tm_zone << ";"
      << "Relative UTC corrected for DST:" << 1 + utcTimeNow.tm_hour - 1 );

    /**
     * Populate the SolTrack structures
     */

    int useDegrees = 1;             // Input (geographic position) and output are in degrees
    int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West w.  0 = North, pi/2 (90deg) = East
    int computeRefrEquatorial = 0;  // Compure refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
    int computeDistance = 1;        // Compute the distance to the Sun in AU: 0-no, 1-yes
      
    struct Time time; // "Time" is probably a pretty dangerous struct name for the SolTrack author to be using

    time.year = 1900 + localTimeNow.tm_year;
    time.month = 1 + localTimeNow.tm_mon;
    time.day = localTimeNow.tm_mday;

    if (localTimeNow.tm_isdst)  // Handle DST
    {
      time.hour = (1 + utcTimeNow.tm_hour) - 1; // UTC!
    }

    time.minute = 1 + localTimeNow.tm_min;
    time.second = 1 + localTimeNow.tm_sec; //SolTrack resolves to micro-seconds, ctime does not. This is good enough.
    
    struct Location loc;

    loc.latitude =  49.2819798;  // 417 Carlen Pl.
    loc.longitude  = -122.82307848;

    loc.pressure = 101.325;  // Atmospheric pressure in kPa @STP, we aren't using refraction anyway (see above)
    loc.temperature = 273.0;  // Atmospheric temperature in K @STP, we aren't using refraction anyway (see above)
    
    // Compute rise and set times:
    struct Position pos;
    struct RiseSet riseSet;
    SolTrack_RiseSet(time, loc, &pos, &riseSet, 0.0, useDegrees, useNorthEqualsZero);  // 5th term (0.0) is altitude (assume meters? It's not clear)
    
    // Compute positions:
    SolTrack(time, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);
    
    /**
     * Populate ROS message with SolTrack structures
     */





    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    std_msgs::Float32 riseTime;
    riseTime.data = riseSet.riseTime;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    solar_position_pub.publish(riseTime);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}