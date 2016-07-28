/* 
 * rosserial Ultrasound Example for HC-SR04   
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

const int echoPin = 5;  //Echo pin
const int trigPin = 6;  //Trigger pin

const int maxRange = 400.0;   //Maximum range in centimeters
const int minRange = 0.0;     //Minimum range

unsigned long range_timer;    //Used to measure 50 ms interval

// instantiate node handle and publisher for 
//  a sensor_msgs/Range message (topic name is /ultrasound)
ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "ultrasound", &range_msg);

/*
 * getRange() - This function reads the time duration of the echo
 *              and converts it to centimeters.
 */
float getRange(){
    int sample;      //Holds time in microseconds
    
    // Trigger pin goes low then high for 10 us then low
    //  to initiate the ultrasonic burst
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // read pulse length in microseconds on the Echo pin
    sample = pulseIn(echoPin, HIGH);
    
    // sample in microseconds converted to centimeters
    // 343 m/s speed of sound;  time divided by 2
    return sample/58.3;
}

char frameid[] = "/ultrasound";   // global frame id string

void setup()
{
  // initialize the node and message publisher
  nh.initNode();
  nh.advertise(pub_range);
  
  // fill the description fields in the range_msg
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.26;
  range_msg.min_range = minRange;
  range_msg.max_range = maxRange;
  
  // set the digital I/O pin modes
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);  
}

void loop()
{
  // sample the range data from the ultrasound sensor and
  // publish the range value once every 50 milliseconds
  if ( (millis()-range_timer) > 50){
    range_msg.range = getRange();
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer =  millis() + 50;
  }
  nh.spinOnce();
}

