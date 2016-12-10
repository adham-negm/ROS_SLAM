#include <ros.h>
#include <ros/time.h>
#include <ros_arduino_base/UpdateGains.h>
#include <ros_arduino_msgs/Encoders.h>
#include <ros_arduino_msgs/CmdDiffVel.h>

// Select your baud rate here
#define BAUD 115200//15200

const int dir_L = 7;
const int en_L = 6;

const int dir_R = 8;
const int en_R = 9; 



#include <Encoder.h>
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder left_encoder(2, 12);
Encoder right_encoder(3, 13);


typedef struct {
  float desired_velocity;     // [m/s]
  uint32_t current_time;      // [milliseconds]
  uint32_t previous_time;     // [milliseconds]
  int32_t current_encoder;    // [counts]
  int32_t previous_encoder;   // [counts]
  float previous_error;       // 
  float total_error;          // 
  int16_t command;            // [PWM]
}
ControlData;


// Vehicle characteristics
float counts_per_rev[1];
float gear_ratio[1];
int encoder_on_motor_shaft[1];
float wheel_radius[1];         // [m]
float meters_per_counts;       // [m/counts]
int pwm_range[1];

// Gains;
float pid_gains[3];
float Kp, Ki, Kd;

// Structures containing PID data
ControlData left_motor_controller;
ControlData right_motor_controller;

// Control methods prototypes
void setupMotors();
void updateControl(ControlData * ctrl, int32_t encoder_reading);
void doControl(ControlData * ctrl);
void Control();
void commandLeftMotor(int16_t cmd);
void commandRightMotor(int16_t cmd);

int control_rate[1];   // [Hz]
int encoder_rate[1];   // [Hz]
int no_cmd_timeout[1]; // [seconds]

uint32_t up_time;             // [milliseconds]
uint32_t last_encoders_time;  // [milliseconds]
uint32_t last_cmd_time;       // [milliseconds]
uint32_t last_control_time;   // [milliseconds]
uint32_t last_status_time;    // [milliseconds]

// ROS node
//<HardwareType, MAX_PUBLISHERS, MAX_SUBSCRIBERS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE>
ros::NodeHandle_<ArduinoHardware, 10, 10, 128, 128> nh;//10 10 1024 1024   280

// ROS subribers/service callbacks prototype
void cmdDiffVelCallback(const ros_arduino_msgs::CmdDiffVel& diff_vel_msg); 

// ROS subsribers
ros::Subscriber<ros_arduino_msgs::CmdDiffVel> sub_diff_vel("cmd_diff_vel", cmdDiffVelCallback);

// ROS services prototype
void updateGainsCb(const ros_arduino_base::UpdateGains::Request &req, ros_arduino_base::UpdateGains::Response &res);
// ROS services
ros::ServiceServer<ros_arduino_base::UpdateGains::Request, ros_arduino_base::UpdateGains::Response> update_gains_server("update_gains", &updateGainsCb);

// ROS publishers msgs
ros_arduino_msgs::Encoders encoders_msg;
char frame_id[] = "base_link";
// ROS publishers
ros::Publisher pub_encoders("encoders", &encoders_msg);


void setup()
{
// Set the node handle
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  
encoders_msg.header.frame_id = frame_id;

  // Pub/Sub
  nh.advertise(pub_encoders);
  nh.subscribe(sub_diff_vel);
  nh.advertiseService(update_gains_server);
  
  // Wait for ROSserial to connect
  while (!nh.connected()) 
  {
    nh.spinOnce();
  }
  nh.loginfo("Connected to microcontroller.");
  
  if (!nh.getParam("control_rate", control_rate,1))
  {
    control_rate[0] = 50;
  }
  if (!nh.getParam("encoder_rate", encoder_rate,1))
  {
    encoder_rate[0] = 50;
  }
  if (!nh.getParam("no_cmd_timeout", no_cmd_timeout,1))
  {
    no_cmd_timeout[0] = 1;
  }
  if (!nh.getParam("pid_gains", pid_gains,3))
  { 
    pid_gains[0] = 35;  // Kp
    pid_gains[1] =   0;  // Ki   1
    pid_gains[2] =  0;  // Kd
  }
  
     

if (!nh.getParam("counts_per_rev", counts_per_rev,1))
  {
    counts_per_rev[0] = 15.0;
  }
  if (!nh.getParam("gear_ratio", gear_ratio,1))
  {
    gear_ratio[0] = 20.0/1.0;
  }
  if (!nh.getParam("encoder_on_motor_shaft", encoder_on_motor_shaft,1))
  {
    encoder_on_motor_shaft[0] = 1;
  }
  if (!nh.getParam("wheel_radius", wheel_radius,1))
  {
    wheel_radius[0] = 0.08/2.0;
  }
  if (!nh.getParam("pwm_range", pwm_range,1))
  {
    pwm_range[0] = 255;
  }
  
  // Compute the meters per count
  if (encoder_on_motor_shaft[0] == 1)
  {
    meters_per_counts = 0.9*((PI * 2 * wheel_radius[0]) / (counts_per_rev[0] * gear_ratio[0]));
  }
  else
  {
    meters_per_counts = 0.9*((PI * 2 * wheel_radius[0]) / counts_per_rev[0]);
  }
  // Create PID gains for this specific control rate
  Kp = pid_gains[0];
  Ki = pid_gains[1] / control_rate[0];
  Kd = pid_gains[2] * control_rate[0];
  

 // Initialize the motors
  setupMotors(); 
  

}

// Initialize encoder 
long positionLeft  = 0;
long positionRight = 0;

void loop()
{
  if ((millis() - last_encoders_time) >= (1000 / encoder_rate[0]))
  { //if encoder publishing readings rate is not met read encoder values and publish them
    encoders_msg.left = left_encoder.read();
    encoders_msg.right = right_encoder.read();
    encoders_msg.header.stamp = nh.now();
    pub_encoders.publish(&encoders_msg);
    last_encoders_time = millis();
  }
  
  if ((millis()) - last_control_time >= (1000 / control_rate[0]))
  {
    Control();
    last_control_time = millis();
  }
  
  // Stop motors after a period of no commands
  if((millis() - last_cmd_time) >= (no_cmd_timeout[0] * 1000))
  {
    left_motor_controller.desired_velocity = 0.0;
    right_motor_controller.desired_velocity = 0.0;
  }
  nh.spinOnce();
  

}

// call backs
void cmdDiffVelCallback( const ros_arduino_msgs::CmdDiffVel& diff_vel_msg) 
{
  left_motor_controller.desired_velocity = diff_vel_msg.left;
  right_motor_controller.desired_velocity = diff_vel_msg.right;
  last_cmd_time = millis();
}

void updateGainsCb(const ros_arduino_base::UpdateGains::Request & req, ros_arduino_base::UpdateGains::Response & res)
{
  for ( int x = 0; x < 3; x++)
  {
    pid_gains[x] = req.gains[x];
  }
  
  Kp = pid_gains[0];
  Ki = pid_gains[1] / control_rate[0];
  Kd = pid_gains[2] * control_rate[0];
}


// motor functions
void setupMotors()
{
  // Initalize Motors
pinMode(dir_L, OUTPUT);
pinMode(en_L, OUTPUT);
  
pinMode(dir_R, OUTPUT);
pinMode(en_R, OUTPUT);
}

void Control()
{
  updateControl(&left_motor_controller, left_encoder.read());
  updateControl(&right_motor_controller, right_encoder.read());

  doControl(&left_motor_controller);
  doControl(&right_motor_controller);

  commandLeftMotor(left_motor_controller.command);
  commandRightMotor(right_motor_controller.command);
  
}

void updateControl(ControlData * ctrl, int32_t encoder_reading)
{
  ctrl->current_encoder = encoder_reading;
  ctrl->current_time = millis();;
}

void doControl(ControlData * ctrl)
{
  float estimated_velocity = meters_per_counts * (ctrl->current_encoder - ctrl->previous_encoder) * 1000.0 / (ctrl->current_time - ctrl->previous_time);
  float error = ctrl->desired_velocity - estimated_velocity;
  float cmd = Kp * error + Ki * (error + ctrl->total_error) + Kd * (error - ctrl->previous_error); //calculate speed correction

  cmd += ctrl->command;

  if(cmd >= pwm_range[0])  // constrain(cmd,pwm_range[0],-pwm_range[0])
  {
    cmd = pwm_range[0];
  }
  else if (cmd <= -pwm_range[0])
  {
    cmd = -pwm_range[0];
  }
 
  ctrl->total_error += error;
  ctrl->command = cmd;
  ctrl->previous_time = ctrl->current_time;
  ctrl->previous_encoder = ctrl->current_encoder;
  ctrl->previous_error = error;

}

void commandLeftMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(dir_L, 0);
  }
  else
  {
    digitalWrite(dir_L, 1);
  }
  analogWrite(en_L, abs(cmd));
}

void commandRightMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(dir_R, 0);
  }
  else
  {
    digitalWrite(dir_R, 1);
  }
  analogWrite(en_R, abs(cmd));
}
