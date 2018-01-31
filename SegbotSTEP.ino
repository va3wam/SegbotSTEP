/*************************************************************************************************************************************
  Segbot Short and Squat, Stepper Motor version (SegbotSTEP)
  This code is used to control a Segbot which is a two wheeled self balancing robot. The PID calculations and IMU logic are based on 
  code and information found on this web site: http://www.brokking.net/yabr_main.html which caused me to change the IMU I was using 
  (switched from Adafruit 9-DOF Absolute Orientation IMU BNO055 to the GY-521 MPU6050 3-Axis Acceleration Gyroscope 6DOF Module - 
  Blue) as well as switching from DC brushed motors to Steppers. 
  
  Networking:
  The ESP8266 acts as a bi-directional, asyncronous socket server. The socket client will either be used for gathering telemetry 
  readings from the SegbotSTEP robot or to issue commands to the robot. The websocket and web code is based on the code found online 
  here: https://gist.github.com/bbx10/667e3d4f5f2c0831d00b. Links to help understand the ESP8266WiFi library can be found here:
  http://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html. This code causes the ESP8266 to act as a web server as 
  well as a websocket server. The use of name vakue pairs as hashmaps is described here: 
  https://www.codeproject.com/Tips/1074030/HashMaps-Associative-Arrays-with-the-Arduino-Progr    

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
  Version MM-DD-YYYY Description
*/   
  String my_ver="1.0";
/*
  ------- ---------- -----------------------------------------------------------------------------------------------------------------
  1.0     01-30-2018 Code base created
 *************************************************************************************************************************************/
#include <ESP8266WiFi.h>                                                     // Connect ESP8266 to AP
#include <ESP8266WiFiMulti.h>                                                // Find best available AP to connect to
#include <WebSocketsServer.h>                                                // https://github.com/Links2004/arduinoWebSockets
#include <Hash.h>                                                            // Allow us to crete associative arrays
#include <ESP8266WebServer.h>                                                // Turn ESP8266 into web server
#include <ESP8266mDNS.h>                                                     // Allow ESP8266 to map services it offers to client 
#include <Wire.h>                                                            // Needed for I2C communication

/*************************************************************************************************************************************
 Define network objects and services
 *************************************************************************************************************************************/
static const char ssid0[] = "MN_BELL418";                                    // The name of a Wi-Fi network AP to connect to
static const char ssid1[] = "MN_WORKSHOP_2.4GHz";                            // The name of a Wi-Fi network AP to connect to
static const char ssid2[] = "MN_DS_OFFICE_2.4GHz";                           // The name of a Wi-Fi network AP to connect to
static const char ssid3[] = "MN_OUTSIDE";                                    // The name of a Wi-Fi network AP to connect to
static const char password[] = "5194741299";                                 // The password for all of the Wi-Fi network APs
static const char *wsEvent[] = { "WStype_DISCONNECTED", "WStype_CONNECTED", "WStype_TEXT", "WStype_BIN"};
MDNSResponder mdns;                                                          // DNS Service Discovery object used for client handshaking
static void writeLED(bool);                                                  // Not sure why this line is needed, function defined below
ESP8266WiFiMulti WiFiMulti;                                                  // Allows us to connect to one of a number of APs.
ESP8266WebServer server(80);                                                 // Define web server object listening on port 80
WebSocketsServer webSocket = WebSocketsServer(81);                           // Define websocket object listening on port 81  

/*************************************************************************************************************************************
 Doug's shortcuts
 *************************************************************************************************************************************/
#define LINE(name,val) Serial.print(name); Serial.print("\t"); Serial.println(val); //Debug macro, prints current code line
#define sp Serial.print                                                      // Shortform print no carrige return
#define spl Serial.println                                                   // Shortform print with carrige return
#define spc Serial.print(", ")                                               // Shortform print comma and space
#define spf Serial.printf                                                    // Shortform formatted printing with no line return

/*************************************************************************************************************************************
 Define on-board LED definitions. GPIO0 is where the onboard LED is located for the Adafruit ESP8266 HUZZAH board. Other board's LED 
 might be on GPIO13.
 *************************************************************************************************************************************/
const int LEDPIN = 0;                                                        // GPIO pin the the onboard LED is connected to
bool LEDStatus;                                                              // Current LED status
const char LEDON[] = "ledon";                                                // Turn onboard LED ON
const char LEDOFF[] = "ledoff";                                              // Turn onboard LED OFF

/*************************************************************************************************************************************
 I2C device address definitions
 *************************************************************************************************************************************/
#define MPU6050_WHO_AM_I 0x75                                                // Read only register on IMU with info about the device
#define MPU_address 0x68                                                     // MPU-6050 I2C address. Note that AD0 pin on the
                                                                             // board cannot be left flaoting and must be connected
                                                                             // to the Arduino ground for address 0x68 or be connected
                                                                             // to VDC for address 0x69.

/*************************************************************************************************************************************
 Define MPU6050 related variables
 *************************************************************************************************************************************/ 
int acc_calibration_value = 1000;                                            // Set this variable to the accelerometer calibration 
                                                                             // value output at start-up of this script
int gyro_pitch_data_raw;                                                     // Collect raw PITCH data from MPU
int gyro_yaw_data_raw;                                                       // Collect raw YAW data from MPU
int accelerometer_data_raw;                                                  // Collect raw ACCELEROMETER data from MPU
int receive_counter;                                                         // Used to get an average off set value for the pitch
int acc_x;                                                                   // Read raw low and high byte to the MPU acc_x register
int acc_y;                                                                   // Read raw low and high byte to the MPU acc_y register
int acc_z;                                                                   // Read raw low and high byte to the MPU acc_z register
int gyro_x;                                                                  // Read raw low and high byte to the MPU gyro_x register
int gyro_y;                                                                  // Read raw low and high byte to the MPU gyro_y register
int gyro_z;                                                                  // Read raw low and high byte to the MPU gyro_z register
int temperature;                                                             // Read raw low and high byte to the MPU temperature register
int tmp;                                                                     // Used to do different calculations to extend the sign bit 
                                                                             // of raw data 
long gyro_yaw_calibration_value;                                             // YAW calibration value
long gyro_pitch_calibration_value;                                           // PITCH calibration value
bool tickOccured;                                                            // Track if ISR has fired
float pid_p_gain = 15;                                                       // Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                                      // Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                                       // Gain setting for the D-controller (30)
float turning_speed = 30;                                                    // Turning speed (20)
float max_target_speed = 150;                                                // Max target speed (100)
float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;               // Gyroscope angle data
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output;       // PID related
float pid_output_left, pid_output_right,pid_last_d_error;                    // PID related

/*************************************************************************************************************************************
 Define power management related variables
 *************************************************************************************************************************************/ 
byte low_bat;                                                                // Flag when battery power level gets too low

/*************************************************************************************************************************************
 Define main loop workflow related variables
 *************************************************************************************************************************************/ 
#define LoopDelay 100000                                                     // This is the target time in milliseconds that each
                                                                             // iteration of Loop() should take. YABR used 4000 but
                                                                             // testing shows that the ESP8266 cannot handle that
                                                                             // speed but it can handle 20000. All angle calcuations
                                                                             // will need to take this into account when porting from
                                                                             // the YABR code base. 
byte start;                                                                  // Flag initial run of main loop
unsigned long loop_timer;                                                    // Used to ensure that each loop() takes the same amount 
                                                                             // of time. The YABR robot uses 4 Milliseconds but tests
                                                                             // indicate that using the ESP8266 Wifi unit will only 
                                                                             // handle 20 Milliseconds. This must be accounted for 
                                                                             // when porting the andle calulations over from YABR.

/*************************************************************************************************************************************
 Declare INDEX_HTML as a FLASH memory string containing a web page. Note that details on programming ESP8266 PROGMEM are found here:
 http://arduino-esp8266.readthedocs.io/en/latest/PROGMEM.html. Details on how to write a Websocket Javascript client can be found
 here: https://www.tutorialspoint.com/websockets/websockets_send_receive_messages.htm
 *************************************************************************************************************************************/
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name = "viewport" content = "width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>SegbotSTEP Web Based Control Center</title>
<style>
"body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }"
</style>
<script>
var websock;
function start() {
  websock = new WebSocket('ws://' + window.location.hostname + ':81/');
  websock.onopen = function(evt) { console.log('websock open'); };
  websock.onclose = function(evt) { console.log('websock close'); };
  websock.onerror = function(evt) { console.log(evt); };
  websock.onmessage = function(evt) {
    console.log(evt);
    var e = document.getElementById('ledstatus');
    if (evt.data === 'ledon') {
      e.style.color = 'red';
    }
    else if (evt.data === 'ledoff') {
      e.style.color = 'black';
    }
    else {
      console.log('unknown event');
    }
  };
}
function buttonclick(e) {
  websock.send(e.id);
}
</script>
</head>
<body onload="javascript:start();">
<h1>ESP8266 WebSocket Demo</h1>
<div id="ledstatus"><b>LED</b></div>
<button id="ledon"  type="button" onclick="buttonclick(this);">On</button> 
<button id="ledoff" type="button" onclick="buttonclick(this);">Off</button>
</body>
</html>
)rawliteral";

/*************************************************************************************************************************************
 This function dumps a bunch of useful info to the terminal. This code is based on an exmaple we found at this URL:
 https://stackoverflow.com/questions/14143517/find-the-name-of-an-arduino-sketch-programmatically                                   
 *************************************************************************************************************************************/
void display_Running_Sketch()
{                                 
  
   sp("[display_Running_Sketch] Sketch Name: ");spl(__FILE__);
   sp("[display_Running_Sketch] Version: "); spl(my_ver);
   sp("[display_Running_Sketch] Compiled on: ");sp(__DATE__);sp(" at ");spl(__TIME__);
   LINE("[display_Running_Sketch] Current line of code test: ", __LINE__);

} //display_Running_Sketch()

/*************************************************************************************************************************************
 This function handles Websocket events. Websockets are persitent bi-directional, asyncronous connections between a client and server  
 *************************************************************************************************************************************/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{

   spf("[webSocketEvent] Event detected: ");                                 // Show event details in terminal   
   spf("num = %d, type = %d (", num, type);                                  // Show event details in terminal
   sp(wsEvent[type-1]);spl(")");                                             // Show event details in terminal
   switch(type)                                                              // Handle each event by type
   {
      case WStype_DISCONNECTED:                                              // Client disconnect event
         spf("[webSocketEvent] [%u] Disconnected!\r\n", num);
         break;
      case WStype_CONNECTED:                                                 // Client connect event
      {
         IPAddress ip = webSocket.remoteIP(num);
         spf("[webSocketEvent] [%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
         // Send the current LED status
         if (LEDStatus)                                                      // If client sent command to change the LED status 
         {
            webSocket.sendTXT(num, LEDON, strlen(LEDON));                    // Send "ledon" string to client
         } //if
         else                                                                // If client wants to turn LED off
         {
            webSocket.sendTXT(num, LEDOFF, strlen(LEDOFF));                  // Send "ledoff" string to client 
         } //else
       } //case
         break;                                                   
      case WStype_TEXT:                                                      // Client sent text event
         spf("[webSocketEvent] Client NUM: [%u], sent TEXT: %s\r\n", num, payload);
         processClientText(num, type, payload, length);
         break;
      case WStype_BIN:                                                       // Client sent binary data event
         spf("[webSocketEvent] [%u] get binary length: %u\r\n", num, length);
         hexdump(payload, length);                                           // Dump ESP8266 memory in hex to the console
         webSocket.sendBIN(num, payload, length);                            // echo data back to browser
         break;
      default:                                                               // Unknown websocket event
         spf("[webSocketEvent] Invalid WStype [%d]\r\n", type);
         break;
   } //switch()

} //webSocketEvent()

/*************************************************************************************************************************************
 This function handles the web server service for valid HTTP GET and POST requests for the root (home) page. The rely simply sends 
 the web page defined in the EEPROM (non-volitile memory of the ESP8266 chip), pointed to by INDEX_HTML[] 
 *************************************************************************************************************************************/
void handleRoot()
{

   server.send_P(200, "text/html", INDEX_HTML);                              // Send the HTML page defined in INDEX_HTML
   spl("[handleRoot] Sent HTML page to client");                              
   
} //handleRoot()

/*************************************************************************************************************************************
 This function handles the web server service for invalid HTTP GET and POST requests. If the requested file or page doesn't exist, 
 return a 404 not found error to the client 
 *************************************************************************************************************************************/
void handleNotFound()
{

   String message = "File Not Found\n\n";                                    // Build string with 404 file not found message
          message += "URI: ";
          message += server.uri();
          message += "\nMethod: ";
          message += (server.method() == HTTP_GET)?"GET":"POST";
          message += "\nArguments: ";
          message += server.args();
          message += "\n";
   for (uint8_t i=0; i<server.args(); i++)                                   // Append actual HTTP error elements to message
   {
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n"; 
   } //for
   server.send(404, "text/plain", message);                                  // Send 404 error message back to client
   spl("[handleNotFound] Sent 404 error to client");                              

} //handleNotFound()

/*************************************************************************************************************************************
 This function controls the onboard Adafruit ESP8266 Huzzah LED. Note inverted logic for Adafruit HUZZAH board
 *************************************************************************************************************************************/
void writeLED(bool LEDon)
{

   LEDStatus = LEDon;                                                        // Track status of LED
   if (LEDon)                                                                // If request is to turn LED on
   {
      digitalWrite(LEDPIN, 0);                                               // Set LED GPIO low, which turn the LED on
      spl("[writeLED] Set LED GPIO LOW (turn LED on)");
   } //if
   else 
   {
      digitalWrite(LEDPIN, 1);                                               // Set LED GPIO high, which turn the LED off
      spl("[writeLED] Set LED GPIO HIGH (turn LED off)");
   } //else

} //writeLED()

/*************************************************************************************************************************************
 This function connects to the local Access Point and then starts up a a socket server to listen for client connections. This code is 
 based on an exmaple we found at this URL: 
 https://stackoverflow.com/questions/14143517/find-the-name-of-an-arduino-sketch-programmatically      
 STATUS return codes: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/include/wl_definitions.h    
 typedef enum {
    WL_NO_SHIELD        = 255 (for compatibility with WiFi Shield library)
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_DISCONNECTED     = 6
} wl_status_t;                         
 *************************************************************************************************************************************/
void startWiFi()
{  

   spl("[startWiFi] Scanning for and connecting to the strongest Access Point signal from a known list...");
   WiFiMulti.addAP(ssid0, password);                                         // Add Wi-Fi AP we may be able to connect to
   WiFiMulti.addAP(ssid1, password);                                         // Add Wi-Fi AP we may be able to connect to
   WiFiMulti.addAP(ssid2, password);                                         // Add Wi-Fi AP we may be able to connect to
   WiFiMulti.addAP(ssid3, password);                                         // Add Wi-Fi AP we may be able to connect to
   while(WiFiMulti.run() != WL_CONNECTED)                                    // Wait for connection to the strongest signalling Wi-Fi AP 
   {                                                               
      sp(".");                                                               // Send dot to console terminal to show the waiting process 
                                                                             // is active
      delay(100);                                                            // Wait a little before trying again
   } //while
   spl("");                                             
   sp("[startWiFi] Connected to Access Point ");                                        
   spl(WiFi.SSID());                                                         // Name of AP to which the ESP8266 is connected to
   sp("[startWiFi] IP address: ");                                   
   spl(WiFi.localIP());                                                      // IP address assigned to ESP8266 by AP

} //startWiFi()

/*************************************************************************************************************************************
 This function Start the mDNS service which handles mapping IP ports and provided services to connecting clients according to
 https://media.readthedocs.org/pdf/arduino-esp8266/docs_to_readthedocs/arduino-esp8266.pdf, mDNS implements a simple DNS server that 
 can be used in both STA and AP modes.  The DNS server currently supports only one domain (for all other domains it will reply with 
 NXDOMAIN or custom status code).  With it, clients can open a web server running on ESP8266 using a domain name, not an IP address.
 *************************************************************************************************************************************/
void startDNS()
{

   if (mdns.begin("espWebSock", WiFi.localIP()))                             // Start mDNS service
   {
      sp("[startDNS] MDNS responder started. ");
      sp("Adding HTTP service to port 80 ");
      mdns.addService("http", "tcp", 80);                                    // If successfully started add web service on port 80
      spl("and WS service to port 81");                    
      mdns.addService("ws", "tcp", 81);                                      // If successfully started add websocket service on port 81
      sp("[startDNS] Clients can connect to either ");
      sp("http://espWebSock.local or http://"); 
      spl(WiFi.localIP());                                                   // Send message to console advising URI for clients to use

   } //if
   else                                                                      // If mDNS service fails to start
   {
      spl("[startDNS] MDNS.begin failed");                                   // Issue message
   } //else
  

} //startDNS()

/*************************************************************************************************************************************
 This function starts the web service which sends HTML and Javascript client code to remote Web browser clients
 *************************************************************************************************************************************/
void startWebServer()
{

   server.on("/", handleRoot);                                               // Attach function for handling root page (/)
   server.onNotFound(handleNotFound);                                        // Attach function for handling unknown page
   server.begin();                                                           // Start web service
   spl("[startWebServer] Started web server");                               

} //startWebServer()

/*************************************************************************************************************************************
 This function starts the Web Socket service which manages bi-directional, asyncronous communication with any web socket clients
 which connect.
 *************************************************************************************************************************************/
void startWebSocketServer()
{

   webSocket.begin();                                                        // Start websocket server
   webSocket.onEvent(webSocketEvent);                                        // Attach function to websocket in order to handle events 
   spl("[startWebSocketServer] Started web socket server");                  

} //startWebSocketServer()

/*************************************************************************************************************************************
 This function processes text messages sent by connected clients
 *************************************************************************************************************************************/
void processClientText(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
   if (strcmp(LEDON, (const char *)payload) == 0)                            // If text sent sets LEDON
   {
      writeLED(true);                                                        // Call function to turn GPIO LED off
   } //if
   else if (strcmp(LEDOFF, (const char *)payload) == 0)                      // If text sent sets LEDOFF 
   {
      writeLED(false);                                                       // Call function to turn GPIO LED on
   } //else if
   else                                                                      // If the text sent is not any of the known commands
   {
      spl("[webSocketEvent] Unknown command");                               // Log unknown command in console
   } //else
   webSocket.broadcastTXT(payload, length);                                  // send payload data to all connected clients
  
} //processClientText()

/*************************************************************************************************************************************
 This function scans the I2C bus for attached devices. This code was taken from http://playground.arduino.cc/Main/I2cScanner 
 *************************************************************************************************************************************/
void startI2Cbus()
{
      
   byte error, address;
   int nDevices;  
   spl("[startI2Cbus] Initialize I2C bus");
   Wire.begin(); 
   spl("[startI2Cbus] Scanning I2C bus...");
   nDevices = 0;
   for(address = 1; address < 127; address++ )
   {
      // The i2c_scanner uses the return value of the Write.endTransmisstion to see if a device did acknowledge to the address
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
     
      if (error == 0)
      {
         sp("[startI2Cbus] Device found at address 0x");
         if (address<16) 
         {
            sp("0");
         } //if
         sp(address,HEX);sp(" - ");
         nDevices++;
         switch(address)                                                     // Check each located device to see what it is
         {
            case MPU_address:                                                // MPU6050
               spl("[startI2Cbus] MPU6050");
               break;
            default:                                                         // Unknown websocket event
               spl("[startI2Cbus] Unknown device");
               break;
         } //switch()          
       } //if
       else if (error==4)
       {
          sp("[startI2Cbus] Unknown error at address 0x");
          if (address<16) 
          {
             sp("0");
          } //if
          spl(address,HEX);spl(" ");
        } //else if    
   } //for
   if (nDevices == 0)
   {
      spl("[startI2Cbus] No I2C devices found");
   } //if
   else
   {
      spl("[startI2Cbus] I2C scan complete");
   } //else

} //startI2Cbus()

/*************************************************************************************************************************************
 This function checks to see if the IUM is at the expected address on the I2C bus. If it is then initialize the IMU. Return codes for 
 speaking to the IMU at endTransmission are as follows:
 0:success
 1:data too long to fit in transmit buffer
 2:received NACK on transmit of address
 3:received NACK on transmit of data
 4:other error
 *************************************************************************************************************************************/
 void initializeIMU()
{

  byte error, lowByte, highByte;
  int address;
  int receive_counter;

  sp("(initializeIMU): Checking to see if the IMU is found at expected I2C address of 0x");
  spl(MPU_address,HEX);
  Wire.beginTransmission(MPU_address);
  error = Wire.endTransmission();

  if (error == 0)
  {

     sp("[initializeIMU] I2C device found at address 0x");
     spl(MPU_address,HEX);
     spl("[initializeIMU] The IMU MPU-6050 found at expected address");
     
     Wire.beginTransmission(MPU_address);
     Wire.write(MPU6050_WHO_AM_I);
     Wire.endTransmission();
     
     spl("[initializeIMU] Send Who am I request to IMU...");
     Wire.requestFrom(MPU_address, 1);
     while(Wire.available() < 1);                                            //Wait for respy from IMU slave on I2C bus                                     
     lowByte = Wire.read();

     if(lowByte == MPU_address)
     {
     
        sp("[initializeIMU] Who Am I responce is ok: 0x");
        spl(lowByte, HEX);
        
        spl("[initializeIMU] Set up the Gyro registers in the IMU");
        set_gyro_registers();
        spl("[initializeIMU] Gyro started and configured");

        spl("[initializeIMU] Wait 10 seconds to allow MPU to settle down");
        delay(10000);

        read_mpu_6050_data();                                                     // Read MPU registers        
        spl("[initializeIMU] If the robopt is on its back then this value is the BALANCE VALUE. TO DO - PUT CODE HERE TO GET BALANCE VALUE");
        //AM: put balance value code here
        
        // Figure out a Gyro offset value for the MPU6050 for this robot. This accounts for errors in the mounting affecting the alu=ignment of the IMU
        spl("[initializeIMU] Create Gyro pitch and yaw offset values by averaging 500 sensor readings...");
        gyro_pitch_calibration_value = 0;                                         // initialize running total of samples

        for(receive_counter = 0; receive_counter < 500; receive_counter++)        //Create 500 loops
        {

           if(receive_counter % 15 == 0) 
           {
              digitalWrite(LEDPIN, !digitalRead(LEDPIN));                      //Change the state of the LED every 15 loops to make the LED blink fast
           } //if
           
           read_mpu_6050_data();                                                  // Read MPU registers  
           // AM: Put logic here to add up the register values we care about      
           delayMicroseconds(300);                                                //Wait for 300 (was 3700) microseconds to simulate the main program loop time

        } //for
   
        // AM: Put code here to get average values for registers
        //gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro pitch offset
        //gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro yaw offset

        //Serial.print("(initializeIMU): Average PITCH gyro offset is ");
        //Serial.println(gyro_pitch_calibration_value/131);
        //Serial.print("(initializeIMU): Average YAW gyro offset is ");
        //Serial.println(gyro_yaw_calibration_value/131);

     } //if
     else
     {
     
        sp("[initializeIMU] Wrong Who Am I responce: 0x");
        if (lowByte<16)Serial.print("0");
        spl(lowByte, HEX);
        spl("[initializeIMU] Initialization of IMU failed");
     
     } //else

  } //if
  else
  {

     sp("[initializeIMU] MPU-6050 query returned error code ");
     spl(error);
     spl("[initializeIMU] ERROR! Robot will not be able to balance");

  } //else
      
} //initializeIMU()

/*************************************************************************************************************************************
 This function configures the MPU6050 using settings recommended by Joop Brokking
 *************************************************************************************************************************************/
void set_gyro_registers()
{

   spl("[set_gyro_registers] Configure the MPU6050...");
   // By default the MPU-6050 sleeps. So we have to wake it up.
   spl("[set_gyro_registers] Wake up MPU");
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search.
   Wire.write(0x6B);                                                         // We want to write to the PWR_MGMT_1 register (6B hex)
   Wire.write(0x00);                                                         // Set the register bits as 00000000 to activate the gyro
   Wire.endTransmission();                                                   // End the transmission with the gyro.

   // Set the full scale of the gyro to +/- 250 degrees per second
   spl("[set_gyro_registers] Set the full scale of the gyro to +/- 250 degrees per second");
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search.
   Wire.write(0x1B);                                                         // We want to write to the GYRO_CONFIG register (1B hex)
   Wire.write(0x00);                                                         // Set the register bits as 00000000 (250dps full scale)
   Wire.endTransmission();                                                   // End the transmission with the gyro
        
   // Set the full scale of the accelerometer to +/- 4g.
   spl("[set_gyro_registers] Set the full scale of the accelerometer to +/- 4g");
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search.
   Wire.write(0x1C);                                                         // We want to write to the ACCEL_CONFIG register (1A hex)
   Wire.write(0x08);                                                         // Set the register bits as 00001000 (+/- 4g full scale range)
   Wire.endTransmission();                                                   // End the transmission with the gyro

   // Set some filtering to improve the raw data.
   spl("[set_gyro_registers] Set Set Digital Low Pass Filter to ~43Hz to improve the raw data");
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search
   Wire.write(0x1A);                                                         // We want to write to the CONFIG register (1A hex)
   Wire.write(0x03);                                                         // Set the register bits as 00000011 (Set Digital Low Pass 
                                                                             // Filter to ~43Hz)
   Wire.endTransmission();                                                   // End the transmission with the gyro 

} //set_gyro_registers

/*************************************************************************************************************************************
 This function reads the accelerometer and gyro info from the MPU6050
 *************************************************************************************************************************************/
void read_mpu_6050_data()                                              
{

  Wire.beginTransmission(MPU_address);                                       // Start communicating with the MPU-6050
  Wire.write(0x3B);                                                          // Send the requested starting register
  Wire.endTransmission();                                                    // End the transmission
  Wire.requestFrom(MPU_address,14);                                          // Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                              // Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                        // Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                        // Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                        // Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                                  // Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_z variable

} //read_mpu_6050_data()

/*************************************************************************************************************************************
 Initialization of all services, subsystems and program execution timing
 *************************************************************************************************************************************/
void setup()
{

   pinMode(LEDPIN, OUTPUT);                                                  // Take control on onbaord LED
   writeLED(false);                                                          // Turn onboard LED off
   Serial.begin(115200);                                                     // Console connection for terminal output
   spl("\r\n[setup] SegbotSTEP sketch starts here");                         // Move to next line of serial trace incase there is noise in console
   display_Running_Sketch();                                                 // Do a dump of a bunch of information to the terminal
   Serial.setDebugOutput(true);                                              // http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html
   for(uint8_t t = 4; t > 0; t--)                                            // Allow time for ESP8266 serial to initialize 
   {
      spf("[setup] Boot wait %d...\r\n", t);                                 // Count down message to console
      Serial.flush();                                                        // Wait for message to clear buffer
      delay(1000);                                                           // Allow time to pass
   } //for
   startWiFi();                                                              // Connect to Access Point
   startDNS();                                                               // Start service directory
   startWebServer();                                                         // Start web service 
   startWebSocketServer();                                                   // Start web socket service
   startI2Cbus();                                                            // Scan the I2C bus for connected devices
   initializeIMU();                                                          // Initialize MPU6050 IMU
   spl("[setup] Initialization of the hardware complete");
   sp("[setup] gyro_pitch_calibration_value= "); 
   spl(gyro_pitch_calibration_value/65.5);
   sp("[setup] gyro_yaw_calibration_value= "); 
   spl(gyro_yaw_calibration_value/65.5);
   loop_timer = micros() + LoopDelay;                                        // Set the loop_timer variable at the next end loop time

} //setup

/*************************************************************************************************************************************
 Main program loop
 *************************************************************************************************************************************/
void loop()
{

   webSocket.loop();                                                         // Poll for websocket client events
   server.handleClient();                                                    // Poll for web server client events
   while(loop_timer > micros());                                             // Ensure that each loop takes the same amount of time                                           
   loop_timer += LoopDelay;                                                  // Increment timer target for next iteration

} //loop

