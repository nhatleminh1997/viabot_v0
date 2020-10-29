/* Arduino - 4 HC-SR04 ultrasonic sensors
 * Author: Nhat Le
 * Last updated: 10/09/2020
 *  * 
 */

#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define SONAR_NUM 4         //Number of utrasonic sensors
#define MAX_DISTANCE 200    //Upper limit for distance
#define PING_INTERVAL 33    //Looping the pings after 33 microseconds

#define SONAR_FRONT_TRIG 8
#define SONAR_FRONT_ECHO 9
#define SONAR_REAR_TRIG 10
#define SONAR_REAR_ECHO 11
#define SONAR_LEFT_TRIG 12
#define SONAR_LEFT_ECHO 13
#define SONAR_RIGHT_TRIG 6
#define SONAR_RIGHT_ECHO 7
#define BAUD 230400

unsigned long pingTimer[SONAR_NUM]; // Times for next ping each sensors
unsigned int cm[SONAR_NUM];         // Results of distance
uint8_t currentSensor = 0;          // Keep track of active sensor

unsigned long _timerStart = 0;

uint8_t oldSensorReading[4];        // Last valid values of sensors

uint8_t frontSensor;
uint8_t rearSensor;
uint8_t leftSensor;
uint8_t rightSensor;

uint8_t frontSensorKalman;
uint8_t rearSensorKalman;
uint8_t leftSensorKalman;
uint8_t rightSensorKalman;

NewPing sonar[SONAR_NUM] = {
  NewPing(SONAR_FRONT_TRIG,SONAR_FRONT_ECHO,MAX_DISTANCE),
  NewPing(SONAR_REAR_TRIG,SONAR_REAR_ECHO,MAX_DISTANCE),
  NewPing(SONAR_LEFT_TRIG,SONAR_LEFT_ECHO,MAX_DISTANCE),
  NewPing(SONAR_RIGHT_TRIG,SONAR_RIGHT_ECHO,MAX_DISTANCE)
};
/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
SimpleKalmanFilter KF_Front(2, 2, 0.01);
SimpleKalmanFilter KF_Rear(2, 2, 0.01);
SimpleKalmanFilter KF_Left(2, 2, 0.01);
SimpleKalmanFilter KF_Right(2, 2, 0.01);

ros::NodeHandle nh;

void sensorCycle(){
  for (uint8_t i = 0; i < SONAR_NUM; i++){
    if (millis() >= pingTimer[i]){
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      //if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor]=0;
      sonar[currentSensor].ping_timer(echoCheck);
      if (currentSensor == SONAR_NUM - 1) oneSensorCycle();
    }
  }
}

// If ping received, set the sensor distance to array.
void echoCheck(){
  if (sonar[currentSensor].check_timer())
  cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//Return the last valid value from the sensor.
void oneSensorCycle() {
  frontSensor  = returnLastValidRead(0, cm[0]);
  rearSensor   = returnLastValidRead(1, cm[1]);
  leftSensor   = returnLastValidRead(2, cm[2]);
  rightSensor  = returnLastValidRead(3, cm[3]);
}

//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}

//Apply Kalman Filter to sensor reading.
void applyKF() {
  frontSensorKalman   = KF_Front.updateEstimate(frontSensor);
  rearSensorKalman    = KF_Rear.updateEstimate(rearSensor);
  leftSensorKalman    = KF_Left.updateEstimate(leftSensor);
  rightSensorKalman   = KF_Right.updateEstimate(rightSensor);
}

void startTimer() {
  _timerStart = millis();
}
 
bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}
 
void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.0;
}

//Create four instances for range messages.
sensor_msgs::Range range_front;
sensor_msgs::Range range_rear;
sensor_msgs::Range range_left;
sensor_msgs::Range range_right;

//Create publisher onjects for all sensors
ros::Publisher pub_range_front("/ultrasound/front", &range_front);
ros::Publisher pub_range_rear("/ultrasound/rear", &range_rear);
ros::Publisher pub_range_left("/ultrasound/left", &range_left);
ros::Publisher pub_range_right("/ultrasound/right", &range_right);

void setup() {
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
 
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();  
  nh.advertise(pub_range_front);
  nh.advertise(pub_range_rear);
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_right);

  sensor_msg_init(range_front,  "/ultrasound/front");
  sensor_msg_init(range_rear,   "/ultrasound/rear");
  sensor_msg_init(range_left,   "/ultrasound/left");
  sensor_msg_init(range_right,  "/ultrasound/right");
}

void loop() {
  sensorCycle();
  oneSensorCycle();
  applyKF();
  range_front.range = frontSensorKalman;
  range_rear.range = rearSensorKalman;
  range_left.range   = leftSensorKalman;
  range_right.range  = rightSensorKalman;
  
  range_front.header.stamp = nh.now();
  range_rear.header.stamp = nh.now();
  range_left.header.stamp = nh.now();
  range_right.header.stamp = nh.now();

  pub_range_front.publish(&range_front);
  pub_range_rear.publish(&range_rear);    
  pub_range_left.publish(&range_left);
  pub_range_right.publish(&range_right);

  startTimer();
  nh.spinOnce();//Handle ROS events
}
