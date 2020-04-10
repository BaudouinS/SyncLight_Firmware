/////TO DO
// - make the multitoggle button works for mode select 
// - make the fader Ff and Fc update to 0 and F0 after pushFf <0 is pushed => try to send an OSC bundle


//DHCP-based OSC server test code
//for use with IDE 1.0.5
//for use with W5100 or W5200 based ethernet shields

//#include <SPI.h>
//#include <Ethernet.h>
//#include <SPI.h>
//#include <EthernetUdp.h>
//#include <OSCBundle.h>


//#ifdef ESP8266
//#include <ESP8266WiFi.h>
//#else
#include <WiFi.h>
//#endif
//#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <Regexp.h> ///https://github.com/nickgammon/Regexp
//#include "SoftwareSerial.h"

//SoftwareSerial mySerial(18, 19); // RX, TX
//SoftwareSerial swSer1;
//#if defined(ESP32)
//HardwareSerial Serial1(2); //UART1/Serial1 pins 16,17
//#endif

//HardwareSerial Serial1( 1 );

//#define Serial1_RX 16
//#define Serial1_TX 17

char ssid[] = "TP-Link_C1BA";
//char ssid[]="HOME-26E2";
char pass[] ="93409582";
//char pass[] ="baked7744borrow";

//char ssid[] = "StrobeWifi";
//char pass[] = "SOE2019";

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
//const IPAddress outIp(10,40,10,105);        // remote IP (not needed for receive)
const unsigned int outPort = 9000;          // remote port (not needed for receive)
const unsigned int localPort = 8000;        // local port to listen for UDP packets (here's where we send the packets)
float F0;
float rangeC=40;
float rangeF=2;
float FCcurrent;
float valueFc;
float valueFf;

OSCErrorCode error;
int ledPin =  13;       //pin 13 on Arduino Uno. Pin 6 on a Teensy++2
unsigned int ledState = LOW;              // LOW means led is *on*
//unsigned int funcPushFfState=LOW;
int funcPushFfState;

int Mode=0;
//float valueF;
//float valueFc;

/////parsing variables
char rxChar;//char variable for serial commands
char rxText[50];
int rxN=0; // Number of chars received
unsigned long count;//for parse function of Serial command. Count the number of axis entered
//////


void setup(){
  pinMode(ledPin, OUTPUT);      
  Serial.begin(115200); //9600 for a "normal" Arduino board (Uno for example). 115200 for a Teensy ++2 
  Serial1.begin(115200);//, SERIAL_8N1, Serial1_RX, Serial1_TX );
  //Serial1.begin(115200, SERIAL_8N1, Serial1_RX, Serial1_TX );
 // mySerial.begin(115200);
// swSer1.begin(115200, 12, 12, SWSERIAL_8N1, false, 256);
// swSer1.enableIntTx(true);
 
 // Serial.println("OSC test");

// Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
#ifdef ESP32
  Serial.println(localPort);
#else
  Serial.println(Udp.localPort());
#endif

Serial1.println("C1;");//send controller on signal
}

void loop(){

  if (Serial1.available()>0){
    // Serial.println("serial com 1 received");

  // what we are searching (the target)
  rxChar = Serial1.read();
  
   // Add normal character
    if(rxChar!='\n'){
      rxText[rxN]=rxChar;
      rxN++;
    }
    // Text is done -> extract speed values as integers and Print results
    if(rxChar=='\n') {
      rxText[rxN]='\0'; // Terminate the string
 //     Serial.print("char read at ");Serial.println(millis());      
      rxN = 0; // Reset the receive counter
      // match state object
          
      
     //Serial.println (rxText);
     
     ///
     MatchState ms;
     ms.Target(rxText) ; 
    // Serial.println(rxText);
    
     ///
     char resultF0=ms.Match ("(F0)(%d+.%d+)(;)",0);

      if (resultF0 == REGEXP_MATCHED){
        char captureBuf [10];
        F0=atof(ms.GetCapture (captureBuf,1)); 
        Serial.print("F0="); Serial.println(F0);

      }
    }
    //delay(100);
  }
  OSCMsgReceive();
} 


void OSCMsgReceive(){
  OSCMessage msgIN;
  int size;
  if((size = Udp.parsePacket())>0){
    while(size--)
      msgIN.fill(Udp.read());
    if(!msgIN.hasError()){
     msgIN.route("/1/OnOffSwitch",toggleOnOff);
      
      msgIN.route("/1/faderH/ValueH",funcValueH);
      msgIN.route("/1/faderS/ValueS",funcValueS);
      msgIN.route("/1/faderV/ValueV",funcValueV);
      
      msgIN.route("/1/faderFc/",funcValueFc);
      msgIN.route("/1/faderFf/",funcValueFf);
//  
      msgIN.route("/1/multitoggleMode/ValMode",MultitoggleMode);
//  

      msgIN.route("/1/pushFf",funcPushFf);
//      msgIN.route("/1/MOVE/ToggleMove");
//      msgIN.route("/1/MOVE/FaderM");
//
//      msgIN.route("/1/Grad/ToggleG");
//      msgIN.route("/1/Grad/FaderG");
//
//      msgIN.route("/1/multitoggleLED/ValLED");
//
//      msgIN.route("/1/Incr");
    }
  }
}

void toggleOnOff(OSCMessage &msg, int addrOffset){
  ledState = (boolean) msg.getFloat(0);
  OSCMessage msgOUT("/1/OnOffSwitch");
  digitalWrite(ledPin, ledState);
   
  msgOUT.add(ledState);
  if (ledState) {
   // Serial.println("LED on");
    Serial1.println("TOF1;");
   // mySerial.print("TOF1;")
    Serial.println("TOF1;");
  }
  else {
   // Serial.println("LED off");
    Serial1.println("TOF0;");
    //mySerial.print("TOF0;")
    Serial.println("TOF0;");
  }

  ledState = !ledState;     // toggle the state from HIGH to LOW to HIGH to LOW ...
  //send osc message back to controll object in TouchOSC
  //Local feedback is turned off in the TouchOSC interface.
  //The button is turned on in TouchOSC interface whe the conrol receives this message.
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message
}

void funcPushFf(OSCMessage &msg, int addrOffset){
  funcPushFfState = 1;//(boolean) msg.getFloat(0);

  OSCMessage msgOUT_PushFf("/1/pushFf");
  msgOUT_PushFf.add(funcPushFfState);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_PushFf.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_PushFf.empty(); // free space occupied by message
  
 // if (funcPushFfState) {
  //valueFc=F0;
  valueFf=0;
   // }
  //Serial1.print("Fc");Serial1.print(valueFc);Serial1.println(";");
  Serial1.print("Ff");Serial1.print(valueFf);Serial1.println(";");
  
  OSCMessage msgOUT_Ff("/1/faderFf/ValueFf");    
  msgOUT_Ff.add(valueFf);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_Ff.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Ff.empty(); // free space occupied by message

//  OSCMessage msgOUT_Fc("/1/faderFc/ValueFc");  
//  msgOUT_Fc.add(valueFc);
//  Udp.beginPacket(Udp.remoteIP(), outPort);
//  msgOUT_Fc.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc.empty(); // free space occupied by message

//  OSCMessage msgOUT_Faderc("/1/faderFc/");
//  msgOUT_Faderc.add(valueFc);
//  Udp.beginPacket(Udp.remoteIP(), outPort);
//  msgOUT_Faderc.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Faderc.empty();
//  
  funcPushFfState = 0;
  
  msgOUT_PushFf.add(funcPushFfState);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_PushFf.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_PushFf.empty(); // free space occupied by message
  
}

void MultitoggleMode(OSCMessage &msg, int addrOffset){
  int Mode = msg.getFloat(0);
  OSCMessage msgOUT("/1/multitoggleMode/ValMode/1");  
  //digitalWrite(ledPin, ledState);
 //Serial.println(Mode);
  msgOUT.add(Mode);

  Mode = !(boolean)Mode;     // toggle the state from HIGH to LOW to HIGH to LOW ...

  //send osc message back to controll object in TouchOSC
  //Local feedback is turned off in the TouchOSC interface.
  //The button is turned on in TouchOSC interface whe the conrol receives this message.
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message
}

void funcValueH(OSCMessage &msg, int addrOffset ){
  int value = msg.getFloat(0);  
  Serial1.print("H");Serial1.print(value);Serial1.println(";");

//  Serial.print("H");Serial1.print(value);Serial1.print(";");
  OSCMessage msgOUT_H("/1/faderH/ValueH"); 
  Serial.print("Value = : ");
  Serial.println(value);
  msgOUT_H.add(value);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_H.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_H.empty(); // free space occupied by message
}

void funcValueS(OSCMessage &msg, int addrOffset ){
  int value = msg.getFloat(0);
  Serial1.print("S");Serial1.print(value);Serial1.println(";");
  OSCMessage msgOUT_S("/1/faderS/ValueS");
  //Serial.print("Value = : ");
  //Serial.println(value);
  msgOUT_S.add(value);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_S.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_S.empty(); // free space occupied by message
}

void funcValueV(OSCMessage &msg, int addrOffset ){
  int value = msg.getFloat(0); 
  Serial1.print("V");Serial1.print(value);Serial1.println(";");
  OSCMessage msgOUT_V("/1/faderV/ValueV");  
  //Serial.print("Value = : ");
  //Serial.println(value);
  msgOUT_V.add(value);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_V.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_V.empty(); // free space occupied by message
}

void funcValueFc(OSCMessage &msg, int addrOffset ){
  valueFc = msg.getFloat(0); 
  //value=((float)value*(40.0/1023.0))-20.0;
  valueFc=F0+((valueFc*rangeC*2.0)/1023.0)-rangeC;
  //valueFc=60+((valueFc*20*2)/1023.0)-20;
  Serial1.print("Fc");Serial1.print(valueFc);Serial1.println(";");
  FCcurrent=valueFc;
  OSCMessage msgOUT_Fc("/1/faderFc/ValueFc");  
 // Serial.print("Value = : ");
 // Serial.println(valueFc);
  msgOUT_Fc.add(valueFc);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_Fc.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc.empty(); // free space occupied by message

  valueFf=0;
  OSCMessage msgOUT_Ff("/1/faderFf/ValueFf");    
  msgOUT_Ff.add(valueFf);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_Ff.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Ff.empty(); // free space occupied by message
  
}

void funcValueFf(OSCMessage &msg, int addrOffset ){
  valueFf = msg.getFloat(0); 
  valueFf=(valueFf*(rangeF*2.0/1023.0))-rangeF; 
  Serial1.print("Ff");Serial1.print(valueFf);Serial1.println(";");
  valueFf=FCcurrent+valueFf;
  OSCMessage msgOUT_Ff("/1/faderFf/ValueFf");  
  msgOUT_Ff.add(valueFf);
  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT_Ff.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Ff.empty(); // free space occupied by message
}
