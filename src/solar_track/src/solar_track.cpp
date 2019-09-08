#include "ros/ros.h"
#include "std_msgs/Float32.h"  //we will have to check on the MCUs capability

extern "C" 
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

  ros::Rate loop_rate(10);  // seconds

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while ( ros::ok() )
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    int useDegrees = 1;             // Input (geographic position) and output are in degrees
    int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
    int computeRefrEquatorial = 1;  // Compure refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
    int computeDistance = 1;        // Compute the distance to the Sun in AU: 0-no, 1-yes
    
    
    struct Time time;
    
    // Set (UT!) date and time manually - use the first date from SolTrack_positions.dat:
    time.year = 2019;
    time.month = 9;
    time.day = 7;
    time.hour = 15;  // 10h PDST + 7 = 15h UT
    time.minute = 0;
    time.second = 0.00;
    
    struct Location loc;
    loc.latitude =  49.2819798;  // 417 Carlen Pl.
    loc.longitude  = -122.82307848;
    loc.pressure = 101.0;     // Atmospheric pressure in kPa
    loc.temperature = 283.0;  // Atmospheric temperature in K
    
    // Compute rise and set times:
    struct Position pos;
    struct RiseSet riseSet;
    SolTrack_RiseSet(time, loc, &pos, &riseSet, 0.0, useDegrees, useNorthEqualsZero);
    
    // Compute positions:
    SolTrack(time, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);
    
    // Write data to screen:
    printf("Date:   %4d %2d %2d\n", time.year, time.month, time.day);
    printf("Time:   %2d %2d %9.6lf\n", (int)time.hour, (int)time.minute, time.second);
    printf("JD:     %20.11lf\n\n", pos.julianDay);
    
    printf("Rise time:      %11.5lf,    azimuth:   %11.5lf\n", riseSet.riseTime, riseSet.riseAzimuth);
    printf("Transit time:   %11.5lf,    altitude:  %11.5lf\n", riseSet.transitTime, riseSet.transitAltitude);
    printf("Set time:       %11.5lf,    azimuth:   %11.5lf\n\n", riseSet.setTime, riseSet.setAzimuth);
    
    printf("Ecliptic longitude, latitude:        %10.6lf° %10.6lf°\n", pos.longitude, 0.0);
    printf("Right ascension, declination:        %10.6lf° %10.6lf°\n", pos.rightAscension, pos.declination);
    printf("Uncorrected altitude:                            %10.6lf°\n\n", pos.altitude);
    printf("Corrected azimuth, altitude:         %10.6lf° %10.6lf°\n", pos.azimuthRefract, pos.altitudeRefract);
    printf("Corected hour angle, declination:    %10.6lf° %10.6lf°\n\n", pos.hourAngleRefract, pos.declinationRefract);
    
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    std_msgs::Float32 riseTime;
    riseTime.data = riseSet.riseTime;

    solar_position_pub.publish(riseTime);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}