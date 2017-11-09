/*******************************************************************************************
  Segbot Short and Squat, Stepper Motor version
  This code is used to control a Segbot which is a two wheeled self balancing robot. Got a
  big boost when I found this web site: http://www.brokking.net/yabr_main.html which caused
  me to change the IMU I was using (switched from Adafruit 9-DOF Absolute Orientation IMU 
  BNO055 to the GY-521 MPU6050 3-Axis Acceleration Gyroscope 6DOF Module - Blue) as well as 
  switching from DC brushed motors to Steppers.

  Physical circuit design
  - Adafruit Feather HUZZAH development board based on ESP8266 @80MHz, 3.3V 4MB flash
  - DC Motor + Stepper Featherwing (https://www.adafruit.com/product/2927) 
  - GY-521 MPU6050 3-Axis Acceleration Gyroscope 6DOF Module - Blue 
  (https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) 
  
  Motor specs
  - Details found online at the RobotShop here: http://www.robotshop.com/ca/en/rbsoy03-soyo-unipolar-stepper-motor.html
  - Manufacturer:CHANGZHOU SONGYANG MACHINERY & ELECTRONICS NEW TECHNIC INSTITUTE
  - Model:SY42STH38-0406A
  - Type: 6 wire Unipolar stepper motors
  - General specifications 
    Resolution expressed in step angle (Degrees): 1.8 (200 steps per revolution of 360 degrees) 
    Torque: 36 oz/inch
    Precision: ±5% 
    Temperature Rise (C): 80 Max (rated curret. 2 phase on)  
    Ambient temperature (C): -20 to +50
    Number of Phase: 2
    Insulation Resistance: 100M Ohms, min (500 VDC)
    Insulation Class: Class B
    Max.radial force (N): 28 (20 mm from flange)
    Max.axial force (N): 10
  - Electrical specifications
    Rated Voltage (V): 12 VDC
    Rated Current (A): 0.4
    Resistance Per Phase (+/- 10% Ohms): 30 (25 C)
    Inductance Per Phase (+/- 20% mH): 30
    Holding Torque (Kg.cm): 2.6 
    Detent Torque (g.cm): 150
    2 Rotor Inertia (g.cm): 54
    Weight (Kg): 0.28
  - Wiring details
    Motor 1: Black (Left), Yellow (center), Green (Right)
    Motor 2: Red (Left), White (center), Blue (Right)   

  IMU specs
  - Model: GY-521
  - Color: Blue
  - Material: PCB + Plastic + copper
  - Chip: MPU-6050
  - Power supply: 3~5V
  - Communication mode: standard IIC communication protocol
  - Chip built-in 16bit AD converter, 16bit data output
  - Gyroscopes range: +/- 250 500 1000 2000 degree/sec
  - Acceleration range: +/- 2g, +/- 4g, +/- 8g, +/- 16g
  - Immersion Gold plating PCB, machine welding process to ensure quality
  - Pin pitch: 2.54mm
  - Great for DIY projects
  - Packing list:
  - 1 x Module
  - 2 x Pins
  - Dimensions: 0.83 in x 0.63 in x 0.12 in (2.1 cm x 1.6 cm x 0.3 cm)
  - Weight: 0.18 oz (5 g)

  Software design
  - Arduino IDE 1.8.2
  - Webserver code based on (http://randomnerdtutorials.com)
  - IMU code based on ...
  - Stepper motor control code based on Adafruit test code that comes with Adafruit 
    assembled Motor Shield for Arduino v2. Adafruit notes that this code won't work 
    with v1.x motor shields, only for the v2's with built in PWM control. See librabry 
    details at http://www.adafruit.com/products/1438

  Development environment
  - Code developed on a MAC OS X Yosemite version 10.10.5 with Arduino IDE version 1.8.2
  - Librabries located at /documents/Arduino/librabries/Adafruit_MotorShield   
  
  History
  MM-DD-YYYY Description
  ---------- -------------------------------------------------------------------------------
  08-02-2017 Code base created.
  08-04-2017 Added proper motor specs (200 steps per revolution or 1.8 degrees per step). 
             Also switched from using Step to oneStep librbary routine to allow for control
             of both motors at the same time by avoiding blocking of I2C bus.
  09-02-2017 Added average pitch and yaw offset value logic based on YABR code found here:
             http://www.brokking.net/yabr_main.html.
  09-03-2017 Added ISR (aka call back routine) to control the stepper motors with more
             precision. This is a software timer interrupt known on the ESP8266 as the OS
             interrupt. It is set to call every 20 milliseconds. Note that this is 1000 
             times SLOWER then the ISR in the YABR code example due to the less powerful
             ESP8266 chip being used here vs the Uno Mini used in the YABR robot. Need to
             modify the angle calculations to reflect this when porting code from YABR.
             Also added loopdelay logic in main loop to reduce range of Loop() execution 
             down from 16K microseconds to 2 microseconds which will make the robots
             movement and angle corrections more consistent. 
  09-03-2017 Added PID calculations from YABR code base. 
  09-04-2017 Swapped out old motor control code that used a direct onestep() library 
             call and replaced it with functions that move a set amount of steps 
             utilizing onestep() internally via the <AccelStepper.h> library. This is done
             to ensure that no delay() or other interrupt blocking code is being used by the 
             library routines which would interfere with the os_timer nor the wifi interrupts.            

  To do:
  - Incorporate YABR code
    - Battery voltage circuit and code
    - Figure out balancing code
    - Add web and/or socket communication for remote control
    - Experiment with motor speeds and step types 
********************************************************************************************/

/////////////////////////////////////////////////////////////////////////////////////////////
// Libraries located relative to root directory
/////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>                                       // Needed for I2C
#include <AccelStepper.h>                               // Needed for motor shield
#include <Adafruit_MotorShield.h>                       // Needed for motor shield
#include "utility/Adafruit_MS_PWMServoDriver.h"         // Needed for motor shield
#include "user_interface.h"                             // Needed for ISR. Note it is reported
                                                        // that microsecond ISRs break Wifi so
                                                        // it may not be possible to have Wifi
                                                        // on this robot using the ESP8266

/////////////////////////////////////////////////////////////////////////////////////////////
// Define constants to help document key values of devices and formulas
/////////////////////////////////////////////////////////////////////////////////////////////
#define MotorShield_I2C_Address 0x60 // I2C address for motorShield. Soldering on board 
                                     // required to change. Dec 96 (hex 0x60) is the default 
                                     // address that we are using.
#define MotorShieldPWMFrequency 1600 // The default (and max) PWM frequency of 1.6KHz. You 
                                     // can specify a lower value then 1600 but not a higher 
                                     // value because this is the upper limit of the PCA9865 
                                     // PWM chip used on the Adafruit Featherwing motor shield. 
#define MotorStepsPerRevolution 200  // 200 steps per revolution (1.8 degrees per step)
#define MotorSpeed 120               // Define the RPMs of the motor (unsigned INT). Max speed 
                                     // is 255 I believe.
#define Motor1 1                     // Refer to the right motor as 1 
#define Motor2 2                     // Refer to the left motor as 2
#define SerialSpeed 9600             // Use slower speed to help with reliability of buses 
#define gyro_address 0x68            // MPU-6050 I2C address. Note that AD0 pin on the
                                     // board cannot be left flaoting and must be connected
                                     // to the Arduino ground for address 0x68 or be connected
                                     // to VDC for address 0x69.
#define MPU6050_WHO_AM_I 0x75        // Read only register on IMU with info about the device
#define OnboardLEDPin 13             // Pin connected to Huzzah board's built-in LED
#define LoopDelay 20000              // This is the target time in milliseconds that each
                                     // iteration of Loop() should take. YABR used 4000 but
                                     // testing shows that the ESP8266 cannot handle that
                                     // speed but it can handle 20000. All angle calcuations
                                     // will need to take this into account when porting from
                                     // the YABR code base. 
#define scopePinISR  2               // Pin used to attach scope to in order to understand ISR timing                                     
#define scopePinLOOP 16               // Pin used to attach scope to in order to understand ISR timing                                     

/////////////////////////////////////////////////////////////////////////////////////////////
// Define global variables
/////////////////////////////////////////////////////////////////////////////////////////////
int acc_calibration_value = 1000;    // Set this variable to the accelerometer calibration 
                                     // value 
int gyro_pitch_data_raw;             // Collect raw PITCH data from MPU
int gyro_yaw_data_raw;               // Collect raw YAW data from MPU
int accelerometer_data_raw;          // Collect raw ACCELEROMETER data from MPU
long gyro_yaw_calibration_value;     // YAW calibration value
long gyro_pitch_calibration_value;   // PITCH calibration value
bool tickOccured;                    // Track if ISR has fired
unsigned long loop_timer;            // Used to ensure that each loop() takes the same amount 
                                     // of time. THe YABR robot uses 4 Milliseconds but tests
                                     // indicate that using the ESP8266 Wifi unit will only 
                                     // handle 20 Milliseconds. This must be accounted for 
                                     // when porting the andle calulations over from YABR.
float pid_p_gain = 15;               //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;              //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;               //Gain setting for the D-controller (30)
float turning_speed = 30;            //Turning speed (20)
float max_target_speed = 150;        //Max target speed (100)

byte lMotorDirection = 0;            // track direction that the left motor should turn (0=forward, 1=backward)
byte rMotorDirection = 0;            // track direction that the right motor should turn (0=forward, 1=backward)

// Do not yet know what these varliables do
int throttle_left_motor;
int throttle_counter_left_motor;
int throttle_left_motor_memory;
int left_motor;

int right_motor;
int throttle_right_motor;
int throttle_counter_right_motor;
int throttle_right_motor_memory;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
byte start;
byte low_bat;
                                     
/////////////////////////////////////////////////////////////////////////////////////////////
// Create ISR object
/////////////////////////////////////////////////////////////////////////////////////////////
os_timer_t myTimer;
                                                                           
/////////////////////////////////////////////////////////////////////////////////////////////
// Create the motor shield object with the default I2C address (0x60). You can change the 
// default address by shorting pins labeled A0 through A4 on the board to chnage the 
// boards address. Shorting each pin adds 1 to the default address. If you do this then you 
// must specify the I2C address of the board when you instantiate it like this example:
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
/////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_MotorShield AFMS = Adafruit_MotorShield(MotorShield_I2C_Address); 
 
/////////////////////////////////////////////////////////////////////////////////////////////
// Create motor objects for both stepper motors used on this robot. Motors have 200 steps per 
// revolution (meaning each step moves 1.8 degrees). Note that the motor numbers are labeled 
// on the MotorShield silkscreen.
/////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_StepperMotor *stepperR = AFMS.getStepper(MotorStepsPerRevolution, Motor1);
Adafruit_StepperMotor *stepperL = AFMS.getStepper(MotorStepsPerRevolution, Motor2);

/////////////////////////////////////////////////////////////////////////////////////////////
// you can change the second argument to SINGLE, DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
/////////////////////////////////////////////////////////////////////////////////////////////
void rightForward() 
{  

   stepperR->onestep(FORWARD, SINGLE);

} //rightForward

/////////////////////////////////////////////////////////////////////////////////////////////
// you can change the second argument to SINGLE, DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
/////////////////////////////////////////////////////////////////////////////////////////////
void rightBackward() 
{  

   stepperR->onestep(BACKWARD, SINGLE);

} //rightBackard

/////////////////////////////////////////////////////////////////////////////////////////////
// you can change the second argument to SINGLE, DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the second motor!
/////////////////////////////////////////////////////////////////////////////////////////////
void leftForward() 
{  

   stepperL->onestep(FORWARD, SINGLE);

} //leftForward

/////////////////////////////////////////////////////////////////////////////////////////////
// you can change the second argument to SINGLE, DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the second motor!
/////////////////////////////////////////////////////////////////////////////////////////////
void leftBackward() 
{  

   stepperL->onestep(BACKWARD, SINGLE);

} //leftBackward

/////////////////////////////////////////////////////////////////////////////////////////////
// Now we'll wrap the 2 steppers in an AccelStepper object
/////////////////////////////////////////////////////////////////////////////////////////////
AccelStepper rightMotor(rightForward, rightBackward);
AccelStepper leftMotor(leftForward, leftBackward);

/////////////////////////////////////////////////////////////////////////////////////////////
// Define ISR (timerCallback)
/////////////////////////////////////////////////////////////////////////////////////////////
void timerCallback(void *pArg) 
{

   digitalWrite(scopePinISR,HIGH);
//   leftForward(); // just put this here to see if we can call this from within the ISR safely
//   rightForward(); // just put this here to see if we can call this from within the ISR safely
  
   

   //Left motor pulse calculations
   throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
   if(throttle_counter_left_motor > throttle_left_motor_memory)              //If the number of loops is larger then the throttle_left_motor_memory variable
   {
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0)                                      //If the throttle_left_motor_memory is negative
    {
//      PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
       lMotorDirection = 1;                                                   //Set left motor left motor direction to backward
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    } //if
//    else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
      else lMotorDirection = 0;                                               //Set left motor left motor direction to forward

  } //if
//  else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
//  else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 
  else if(throttle_counter_left_motor == 1)
  {
//    PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
     if(lMotorDirection = 0)
     {
        leftForward(); 
     } //if
     else
     { 
        leftBackward(); 
     } //else
  } //elseif

  
/*
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
*/

   digitalWrite(scopePinISR,LOW);

} // timerCallback()

/////////////////////////////////////////////////////////////////////////////////////////////
// Create ISR based on the ESP8266 timer and definition above
/////////////////////////////////////////////////////////////////////////////////////////////
void create_ISR(void) 
{
 /*------------------------------------------------------------------------------------------ 
    os_timer_setfn - Define a function to be called when the timer fires
    void os_timer_setfn(
      os_timer_t *pTimer,
      os_timer_func_t *pFunction,
      void *pArg)

    Define the callback function that will be called when the timer reaches zero. The 
    pTimer parameters is a pointer to the timer control structure.
    The pFunction parameters is a pointer to the callback function.
    The pArg parameter is a value that will be passed into the called back function. The 
    callback function should have the signature:
    void (*functionName)(void *pArg)
    The pArg parameter is the value registered with the callback function.
   -----------------------------------------------------------------------------------------*/
   os_timer_setfn(&myTimer, timerCallback, NULL);

 /*------------------------------------------------------------------------------------------ 
   os_timer_arm -  Enable a millisecond granularity timer.
   void os_timer_arm(
      os_timer_t *pTimer,
      uint32_t milliseconds,
      bool repeat)
   Arm a timer such that is starts ticking and fires when the clock reaches zero.
   The pTimer parameter is a pointed to a timer control structure.
   The milliseconds parameter is the duration of the timer measured in milliseconds. The 
   repeat parameter is whether or not the timer will restart once it has reached zero.
   -----------------------------------------------------------------------------------------*/
   os_timer_arm(&myTimer, 20, true);

} //create_ISR

/////////////////////////////////////////////////////////////////////////////////////////////
// Boot up routine for robot
/////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{

   // Initialize I2C bus
   Wire.begin();
  
   // Initialize console terminal
   Serial.begin(SerialSpeed);
   delay(100);  

   // Allow onboard LED to be controlled by this program
   pinMode(OnboardLEDPin, OUTPUT);
           
   Serial.println("*** Ignore noise in terminal prior to this point...");
   Serial.println("(setup): Initialize SegbotSTEP hardware...");

   //Set up ISR
   pinMode(scopePinISR,OUTPUT);  // Used to test ISR timing with a scope   
   pinMode(scopePinLOOP,OUTPUT); // Used to test ISR timing with a scope   
   tickOccured = false;
   Serial.println("(setup): Create ISR callback routine to control stepper motors...");
   create_ISR();

   // Initialize MPU6050 IMU
   Serial.println("(setup): Initialize IMU");
   initializeIMU();

   // Initialize Adafruit MotorShield
   Serial.println("(setup): Initialize MotorShield");
   initializeMotorShield();

   Serial.println("(setup): Initialization of the SegbotSTEP hardware complete");

   //Set the loop_timer variable at the next end loop time
   loop_timer = micros() + LoopDelay;
  
} //setup

/////////////////////////////////////////////////////////////////////////////////////////////
// This is the main program logic. 
/////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
    
   digitalWrite(scopePinLOOP,!digitalRead(scopePinLOOP));
   
   //-------------------------------------------------------------------------------------------------------------------------------------
   // Angle calculations
   //-------------------------------------------------------------------------------------------------------------------------------------
   Wire.beginTransmission(gyro_address);                                     // Start communication with the gyro
   Wire.write(0x3F);                                                         // Start reading at register 3F
   Wire.endTransmission();                                                   // End the transmission
   Wire.requestFrom(gyro_address, 2);                                        // Request 2 bytes from the gyro
   accelerometer_data_raw = Wire.read()<<8|Wire.read();                      // Combine the two bytes to make one integer
   accelerometer_data_raw += acc_calibration_value;                          // Add the accelerometer calibration value
   if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           // Prevent division by zero by limiting the acc data to +/-8200;
   if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         // Prevent division by zero by limiting the acc data to +/-8200;

   angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           // Calculate the current angle according to the accelerometer

   if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5)                      // If the accelerometer angle is almost 0
   {
    
     angle_gyro = angle_acc;                                                 // Load the accelerometer angle in the angle_gyro variable
     start = 1;                                                              // Set the start variable to start the PID controller
  
   } //if
  
   Wire.beginTransmission(gyro_address);                                     // Start communication with the gyro
   Wire.write(0x43);                                                         // Start reading at register 43
   Wire.endTransmission();                                                   // End the transmission
   Wire.requestFrom(gyro_address, 4);                                        // Request 4 bytes from the gyro
   gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           // Combine the two bytes to make one integer
   gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         // Combine the two bytes to make one integer
  
   gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      // Add the gyro calibration value
   angle_gyro += gyro_pitch_data_raw * 0.000031;                             // Calculate the traveled during this loop angle and add this to the angle_gyro variable
  
   //-------------------------------------------------------------------------------------------------------------------------------------
   // MPU-6050 offset compensation
   //-------------------------------------------------------------------------------------------------------------------------------------
   // Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
   // As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
   // To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
   // Try 0.0000003 or -0.0000003 first to see if there is any improvement.

   gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          // Add the gyro calibration value
   // Uncomment the following line to make the compensation active
   //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            // Compensate the gyro offset when the robot is rotating

   angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    // Correct the drift of the gyro angle with the accelerometer angle

   //-------------------------------------------------------------------------------------------------------------------------------------
   // PID controller calculations
   //-------------------------------------------------------------------------------------------------------------------------------------
   // The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
   // is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
   // The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
   pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
   if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015;

   pid_i_mem += pid_i_gain * pid_error_temp;                                 // Calculate the I-controller value and add it to the pid_i_mem variable
   if(pid_i_mem > 400)pid_i_mem = 400;                                       // Limit the I-controller to the maximum controller output
   else if(pid_i_mem < -400)pid_i_mem = -400;
  
   //Calculate the PID output value
   pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
   if(pid_output > 400)pid_output = 400;                                     // Limit the PI-controller to the maximum controller output
   else if(pid_output < -400)pid_output = -400;

   pid_last_d_error = pid_error_temp;                                        // Store the error for the next loop

   if(pid_output < 5 && pid_output > -5)pid_output = 0;                      // Create a dead-band to stop the motors when the robot is balanced

   if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1)     // If the robot tips over or the start variable is zero or the battery is empty
   {
  
      pid_output = 0;                                                        // Set the PID controller output to 0 so the motors stop moving
      pid_i_mem = 0;                                                         // Reset the I-controller memory
      start = 0;                                                             // Set the start variable to 0
      self_balance_pid_setpoint = 0;                                         // Reset the self_balance_pid_setpoint variable
  
    } //if

   //-------------------------------------------------------------------------------------------------------------------------------------
   // Current motor control logic, to be replaced by converted code from YABR
   // Here are notes from the librabry notes that help remind me why I want to use
   // the oneStep() function and not the (step) function used in the example code from Adafruit:
   // 
   // The oneStep(DIRECTION, STYLE) function is a low-level internal function called by step(). 
   // But it can be useful to call on its own to implement more advanced functions such as 
   // acceleration or coordinating simultaneous movement of multiple stepper motors. The 
   // direction and style parameters are the same as for step(), but onestep() steps exactly 
   // once. Note: Calling step() with a step count of 1 is not the same as calling onestep(). 
   // The step function has a delay based on the speed set in setSpeed(). onestep() has no delay. 
   //
   // DIRECTION = FORWARD or BACKWARD
   // STYLE = SINGLE, DOUBLE, INTERLEAVE or MICROSTEP 
   //-------------------------------------------------------------------------------------------------------------------------------------
/*
   // Change direction of stepper 1 if at the set limit
   if (rightMotor.distanceToGo() == 0)
   {
  
      rightMotor.moveTo(-rightMotor.currentPosition());
   
   } //if
   
   // Change direction of stepper 2 if at the set limit
   if (leftMotor.distanceToGo() == 0)
   {
    
   leftMotor.moveTo(-leftMotor.currentPosition());
   
   } //if

   rightMotor.run();
   leftMotor.run();

*/

   //-------------------------------------------------------------------------------------------------------------------------------------
   //Motor pulse calculations
   //-------------------------------------------------------------------------------------------------------------------------------------
   //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
   if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
   else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

   if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
   else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

   //Calculate the needed pulse time for the left and right stepper motor controllers
   if(pid_output_left > 0)left_motor = 400 - pid_output_left;
   else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
   else left_motor = 0;

   if(pid_output_right > 0)right_motor = 400 - pid_output_right;
   else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
   else right_motor = 0;

   //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
   throttle_left_motor = left_motor;
   throttle_right_motor = right_motor;

   //-------------------------------------------------------------------------------------------------------------------------------------
   // Loop time timer. The angle calculations are tuned for a loop time of 20 milliseconds. To make sure every loop is exactly 20 
   // milliseconds a wait loop is created by setting the loop_timer variable to LoopDelay (+20000 microseconds) every loop.
   //-------------------------------------------------------------------------------------------------------------------------------------
   while(loop_timer > micros());
   loop_timer += LoopDelay;

   //-------------------------------------------------------------------------------------------------------------------------------------
   // Test print lines used to see what values are being carried in key variables or system functions
   // Typically planned to only uncomment 1 at most at a time during analysis and troubleshooting
   //-------------------------------------------------------------------------------------------------------------------------------------
   //Serial.println(micros());   
   Serial.println(angle_gyro); 
   Serial.println(angle_acc);   
      
} //loop()

/////////////////////////////////////////////////////////////////////////////////////////////
// Check if the MotorShield is at the expected address on the I2C bus and if so initialize 
// it.
/////////////////////////////////////////////////////////////////////////////////////////////
void initializeMotorShield()
{

  byte error;

  Serial.print("(initializeMotorShield): Checking to see if the MotorShield is found at expected I2C address of 0x");
  Serial.println(MotorShield_I2C_Address,HEX);

  Wire.beginTransmission(MotorShield_I2C_Address);
  error = Wire.endTransmission();

  if (error == 0)
  {

     Serial.print("(initializeMotorShield): I2C device found at address 0x");
     Serial.println(MotorShield_I2C_Address,HEX);
     Serial.println("(initializeMotorShield): MotorShield found at expected address");
     Serial.print("(initializeMotorShield): Instantiating MotorShield object with a PWM of ");
     Serial.println(MotorShieldPWMFrequency);
     AFMS.begin(MotorShieldPWMFrequency);
  
     Serial.print("(initializeMotorShield): Default motor settings are ");
     Serial.print("MAX SPEED: 100.0, ");
     Serial.print("ACCELERATION: 100.0, ");
     Serial.println("MOVE TO: 50, ");
     
     rightMotor.setMaxSpeed(100.0);
     rightMotor.setAcceleration(100.0);
     rightMotor.moveTo(50);
    
     leftMotor.setMaxSpeed(100.0);
     leftMotor.setAcceleration(100.0);
     leftMotor.moveTo(50);

  } //if
  else
  {

     Serial.print("(initializeMotorShield): Motorshield query returned error code ");
     Serial.println(error);
     Serial.println("(initializeMotorShield): ERROR! Robot will not be able to move. Try powering everything off, have seen error code 4 cleared this way.");

  } //else
         
} //initializeMotorShield()

/////////////////////////////////////////////////////////////////////////////////////////////
// Check if the IUM is at the expected address on the I2C bus. If it is then initialize
// the IMU. Return codes for speaking to the IMU at endTransmission are as follows:
//
// 0:success
// 1:data too long to fit in transmit buffer
// 2:received NACK on transmit of address
// 3:received NACK on transmit of data
// 4:other error 
/////////////////////////////////////////////////////////////////////////////////////////////
void initializeIMU()
{

  byte error, lowByte, highByte;
  int address;
  int receive_counter;

  Serial.print("(initializeIMU): Checking to see if the IMU is found at expected I2C address of 0x");
  Serial.println(gyro_address,HEX);
  Wire.beginTransmission(gyro_address);
  error = Wire.endTransmission();

  if (error == 0)
  {

     Serial.print("(initializeIMU): I2C device found at address 0x");
     Serial.println(gyro_address,HEX);
     Serial.println("(initializeIMU): The IMU MPU-6050 found at expected address");
     
     Wire.beginTransmission(gyro_address);
     Wire.write(MPU6050_WHO_AM_I);
     Wire.endTransmission();
     
     Serial.println("(initializeIMU): Send Who am I request to IMU...");
     Wire.requestFrom(gyro_address, 1);
     while(Wire.available() < 1); //Wait for respy from IMU slave on I2C bus                                     
     lowByte = Wire.read();

     if(lowByte == 0x68)
     {
     
        Serial.print("(initializeIMU): Who Am I responce is ok: 0x");
        Serial.println(lowByte, HEX);
        
        Serial.println("(initializeIMU): Set up the Gyro registers in the IMU");
        set_gyro_registers();
        Serial.println("(initializeIMU): Gyro started and configured");

        Serial.print("(initializeIMU): Balance value of Robot (only valid if robot is upright on a stand): ");
        Wire.beginTransmission(gyro_address); //Start communication with the IMU
        Wire.write(0x3F); // Get the MPU6050_ACCEL_ZOUT_H value
        Wire.endTransmission(); //End the transmission with the gyro
        Wire.requestFrom(gyro_address,2);
        Serial.println((Wire.read()<<8|Wire.read())*-1);
        delay(20);

        Serial.println("(initializeIMU): Set pitch and yaw offset values...");
       
        for(receive_counter = 0; receive_counter < 500; receive_counter++)  //Create 500 loops
        {
          
           if(receive_counter % 15 == 0)
           {
              digitalWrite(OnboardLEDPin, !digitalRead(OnboardLEDPin));     //Change the state of the LED every 15 loops 
                                                                            //to make the LED blink fast
           } //if
           
           Wire.beginTransmission(gyro_address);                            //Start communication with the gyro
           Wire.write(0x43);                                                //Start reading the Who_am_I register 75h
           Wire.endTransmission();                                          //End the transmission
           Wire.requestFrom(gyro_address, 4);                               //Request 2 bytes from the gyro
           gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();        //Combine the two bytes to make one integer
           gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();      //Combine the two bytes to make one integer
           delay(0);                                                        //Slow down I2C commands to allow the Wifi stack to do its thing. 
                                                                            //Lots of info on the internet plus my own experience with this 
                                                                            //chip shows that this is needed to ensure stability.
                                                                            
        } //for
  
        gyro_pitch_calibration_value /= 500;                                //Divide the total value by 500 to get the avarage gyro offset
        gyro_yaw_calibration_value /= 500;                                  //Divide the total value by 500 to get the avarage gyro offset
        Serial.print("(initializeIMU): Average PITCH gyro offset is ");
        Serial.println(gyro_pitch_calibration_value);
        Serial.print("(initializeIMU): Average YAW gyro offset is ");
        Serial.println(gyro_yaw_calibration_value);

     } //if
     else
     {
     
        Serial.print("(initializeIMU): Wrong Who Am I responce: 0x");
        if (lowByte<16)Serial.print("0");
        Serial.println(lowByte, HEX);
        Serial.println("(initializeIMU): Initialization of IMU failed");
     
     } //else

  } //if
  else
  {

     Serial.print("(initializeIMU): MPU-6050 query returned error code ");
     Serial.println(error);
     Serial.println("(initializeIMU): ERROR! Robot will not be able to balance");

  } //else
  
        
} //initializeIMU()

/////////////////////////////////////////////////////////////////////////////////////////////
// Setup the MPU-6050 
/////////////////////////////////////////////////////////////////////////////////////////////
void set_gyro_registers()
{

  Wire.beginTransmission(gyro_address);   //Start communication with the IMU
  Wire.write(0x6B);                       //Write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                       //Set the register bits to 00000000 to activate the gyro
  Wire.endTransmission();                 //End the transmission with the gyro
  
  Wire.beginTransmission(gyro_address);   //Start communication with the IMU
  Wire.write(0x1B);                       //Write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                       //Set register bits to 00000000 (250dps full scale)
  Wire.endTransmission();                 //End the transmission with the gyro

  Wire.beginTransmission(gyro_address);   //Start communication with the IMU
  Wire.write(0x1C);                       //Write to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x08);                       //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                 //End the transmission with the gyro

  Wire.beginTransmission(gyro_address);   //Start communication with the IMU
  Wire.write(0x1A);                       //Write to the CONFIG register (1A hex)
  Wire.write(0x03);                       //Set the register bits to 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                 //End the transmission with the gyro

} //set_gyro_registers




