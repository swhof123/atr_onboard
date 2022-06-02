// ########################## DEFINES ##########################
//Imports
#include <WiFi.h>        // Web-Socket


// Hoverboard communication
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD      // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval (Original 100)
// #define DEBUG_RX                     // [-] Debug received data. Prints all bytes to serial (comment-out to disable, also needs to be activated in config.h on hoverboard)

HardwareSerial HoverSerial(2);          // Using UART 2 with RX on pin  16, TX on pin 17
#define RXD2 17
#define TXD2 16

//Second Hoverboard
HardwareSerial HoverSerial1(1);          // Using UART 1 with RX on pin  2, TX on pin 4 for second hoverboard
#define RXD1 2
#define TXD1 4

// Hoverboard setup
#define SPEED_MAX  100                  // [-] Maximum speed forward, can be upt to 1000
#define SPEED_MIN -100                  // [-] Maximum speed backwards, can be upt to -1000
#define SPEED_INCREMENT 10
#define STEER_MAX 100                  // [-] Maximum steer, can be upt to 1000
#define STEER_MIN -100                 // [-] Maximum steer, can be upt to -1000
#define STEER_INCREMENT 5
#define STEER_INCREMENT_SLOW 20


//Mower Commands
int steerVal = 0;
int speedVal = 0;
int steerVal1 = 0;
int speedVal1 = 0;
bool underControl = 0;

//Mower Motor
bool mower = 0;
const int relayPin = 23;


// Hoverbaord messages definition
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

uint8_t idx1 = 0;                        // Index for new data pointer
uint16_t bufStartFrame1;  
byte *p1;                                // Pointer declaration for the new received data
byte incomingByte1;
byte incomingBytePrev1;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;
SerialCommand Command1;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   //int16_t  distL;
   //int16_t  distR;
   //int16_t  warning;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;
SerialFeedback Feedback1;
SerialFeedback NewFeedback1;

String warning_message = "";
String warning_message1 = "";



unsigned long timeSend = 0;             

//WiFi Setup
const char *ssid = "iPhone von Georg";  //Change to iPhone
const char *password = "um7uftmz6xc16";

//Server setup
int port = 8888;
WiFiServer server(port);
#define TIME_SEND_WS 500      //Websocket updates in [ms] 
unsigned long timeSendWs = 0;
unsigned long lastConnection = 0;     

// ########################## WiFi ##########################
void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Successfully connected to Access Point");
}

void Get_IPAddress(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WIFI is connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WIFI access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Reconnecting...");
  WiFi.begin(ssid, password);
}

// ########################## Websocket ##########################
 String notifyClients() {
  String cmd1 = String(Feedback.cmd1);
  String cmd2 = String(Feedback.cmd2);
  String speedR_meas = String(Feedback.speedR_meas);
  String speedL_meas = String(Feedback.speedL_meas);
  String batVoltage = String(Feedback.batVoltage);
  String boardTemp = String(Feedback.boardTemp);
  //String distL = String(Feedback.distL);
  //String distR = String(Feedback.distR);

  String cmd11 = String(Feedback1.cmd1);
  String cmd21 = String(Feedback1.cmd2);
  String speedR_meas1 = String(Feedback1.speedR_meas);
  String speedL_meas1 = String(Feedback1.speedL_meas);
  String batVoltage1 = String(Feedback1.batVoltage);
  String boardTemp1 = String(Feedback1.boardTemp);
  //String distL1 = String(Feedback1.distL);
  //String distR1 = String(Feedback1.distR);

  String state = "{\"board0\":{\"steeringcmd\":" + cmd1 +",\"speedcmd\":" + cmd2 + ",\"speedR\":" + speedR_meas 
                  + ",\"speedL\":" + speedL_meas + ",\"batVoltage\":" + batVoltage
                  + ",\"boardTemp\":" + boardTemp + "},\"board1\":{\"steeringcmd\":"
                  + cmd11 +",\"speedcmd\":" + cmd21 + ",\"speedR\":" + speedR_meas1 
                  + ",\"speedL\":" + speedL_meas1 + ",\"batVoltage\":" + batVoltage1
                  + ",\"boardTemp\":" + boardTemp1 + "},\"mower\":" + mower +"}\r\n";

  //String state = "{\"steeringcmd\":" + cmd1 +",\"speedcmd\":" + cmd2 + ",\"speedR\":" + speedR_meas 
  //                + ",\"speedL\":" + speedL_meas + ",\"batVoltage\":" + batVoltage
  //                + ",\"boardTemp\":" + boardTemp + ",\"distL\":" + distL + ",\"distR\":" + distR + ",\"motor_warning\":" + "\"" + warning_message + "\"" + ",\"mower\":" + mower + ",\"cmdLed\":" + cmdLed + "}";
  // Serial.print("notifyClent with:");Serial.println(state); //<-- Clutters the serial output

  return state;
}

void Stopping() {
  steerVal = 0;
  steerVal1 = 0;
  speedVal = 0;
  speedVal1 = 0;
  mower = 0;
  // Serial.print("All Stopped: Steering: ");Serial.print(steerVal);Serial.print(", Speed: ");Serial.println(speedVal); //<-- Clutters the serial output
}
  
void Send(int16_t uSteer, int16_t uSpeed) {
  //Hoverboard 0
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 

}

void Send1(int16_t uSteer, int16_t uSpeed) {
  //Hoverboard 1
  //Create command
  Command1.start    = (uint16_t)START_FRAME;
  Command1.steer    = (int16_t)uSteer;
  Command1.speed    = (int16_t)uSpeed;
  Command1.checksum = (uint16_t)(Command1.start ^ Command1.steer ^ Command1.speed);

  //Write to Serial
  HoverSerial1.write((uint8_t *) &Command1, sizeof(Command1)); 

}


void Receive() {
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte     = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;  
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    } 
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                             ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
            //Serial.println("Valid Input received"); //<-- Clutters the serial output
        } else {
          //Serial.println("Non-valid data skipped"); //<-- Clutters the serial output
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void Receive1() {
    // Check for new data availability in the Serial buffer
    if (HoverSerial1.available()) {
        incomingByte1     = HoverSerial1.read();                                   // Read the incoming byte
        bufStartFrame1 = ((uint16_t)(incomingByte1) << 8) | incomingBytePrev1;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX1
        Serial.print(incomingByte1);
        return;
    #endif

    // Copy received data
    if (bufStartFrame1 == START_FRAME) {                     // Initialize if new data is detected
        p1       = (byte *)&NewFeedback1;
        *p1++    = incomingBytePrev1;
        *p1++    = incomingByte1;
        idx1     = 2;  
    } else if (idx1 >= 2 && idx1 < sizeof(SerialFeedback)) {  // Save the new received data
        *p1++    = incomingByte1; 
        idx1++;
    } 
    
    // Check if we reached the end of the package
    if (idx1 == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback1.start ^ NewFeedback1.cmd1 ^ NewFeedback1.cmd2 ^ NewFeedback1.speedR_meas ^ NewFeedback1.speedL_meas
                            ^ NewFeedback1.batVoltage ^ NewFeedback1.boardTemp ^ NewFeedback1.cmdLed);

        // Check validity of the new data
        if (NewFeedback1.start == START_FRAME && checksum == NewFeedback1.checksum) {
            // Copy the new data
            memcpy(&Feedback1, &NewFeedback1, sizeof(SerialFeedback));
        } else {
          //Serial.println("Non-valid data skipped"); //<-- Clutters the serial output
        }
        idx1 = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev1 = incomingByte1;
}

// ########################## SETUP ##########################
void setup() {

  //Serial Output
  Serial.begin(SERIAL_BAUD);
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
  HoverSerial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD1, TXD1);
  Serial.println();

  // Wifi Setup  
  WiFi.disconnect(true);
  delay(1000);

  //Establishing the Wifi Events
  WiFi.onEvent(Wifi_connected,SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(Get_IPAddress, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, SYSTEM_EVENT_STA_DISCONNECTED); 
  WiFi.begin(ssid, password);
  Serial.println("Waiting for WIFI network...");
  delay(1000);

  //Websocket server
  server.begin();

  Serial.println("Server started");

  //Mower motor
  pinMode(relayPin, OUTPUT);
}


// ########################## LOOP ##########################

void loop() {
  // Receive data from motor controllers
  Receive();
  Receive1();

  unsigned long timeNow = millis();

  if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
    Serial.println("IP Address Issue! Restarting");
    ESP.restart();
    };

  // listen for rover main unit
  WiFiClient client = server.available();

  if (client) {

    if (client.connected()) {
      
      Serial.println("Client connected");
      lastConnection = timeNow;
      underControl = 1;

      //if (client.available() > 0) {
      if (1 == 1){ // Hack! for some reason, the client.available does not work...        
  
        //Forward motor controller data to main unit
        if (timeSendWs < timeNow) { 
        Serial.println("Sending data");
        String msg = notifyClients();   
        char *cmsg = &msg[0];
        client.write(cmsg);
        timeSendWs = timeNow + TIME_SEND_WS;
        }
  
        //Read data from main processor
        Serial.println("Data to be read");
        steerVal = client.readStringUntil('#').toInt();
        steerVal1 = client.readStringUntil('#').toInt();
        speedVal = client.readStringUntil('#').toInt();
        speedVal1 = client.readStringUntil('#').toInt();
        mower = client.readStringUntil('\n').toInt();        
        Serial.print("steerVal: "); Serial.println(steerVal);
        Serial.print("steerVal1: "); Serial.println(steerVal1);
        Serial.print("speedVal: "); Serial.println(speedVal);
        Serial.print("speedVal1: "); Serial.println(speedVal1);
        Serial.print("motorVal: "), Serial.println(mower);
        lastConnection = timeNow;
        
      } else {
        Serial.println("No clients available");
        underControl = 0;
      }
    } else {
      Serial.println("Disconnecting");
      client.stop();  
      underControl = 0;
    }
  } 

  if (timeNow > lastConnection + 4 * TIME_SEND_WS) {
    Serial.println("No update for too long! Stopping!");  
    underControl = 0;
    lastConnection = timeNow;
  }    
  
  
  // Send commands from main unit to motor controller
  if (timeSend < timeNow) {
    if (underControl == 0) {
      Stopping();
    }
    //Mower Motor
    if (mower == 1) {
      Serial.println("Mower on");
      digitalWrite(relayPin, HIGH);
    } else {
      digitalWrite(relayPin, LOW);
    }        
    Send(steerVal, speedVal);
    Send1(steerVal1, speedVal1);
    timeSend = timeNow + TIME_SEND;
  }
  delay(1);
}
