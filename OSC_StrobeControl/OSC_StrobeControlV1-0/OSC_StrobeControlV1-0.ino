//DHCP-based OSC server test code
//for use with IDE 1.0.5
//for use with W5100 or W5200 based ethernet shields

#include <SPI.h>
#include <Ethernet.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>

// you can find this written on the board of some Arduino Ethernets or shields
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x26, 0xEA };
//byte mac[] = { 0xD8, 0x0D, 0x17, 0x5D, 0xC1, 0xBA };

  //90:C7:92:58:26:E0
// NOTE: Alternatively, you can assign a fixed IP to configure your
//       Ethernet shield.
      byte ip[] = { 192, 168, 0, 1 };
 //  byte ip[] ={ 192, 168, 0, 100 };
int serverPort  = 8000; //TouchOSC (incoming port)
int destPort = 9000;    //TouchOSC (outgoing port)
int ledPin =  12;       //pin 13 on Arduino Uno. Pin 6 on a Teensy++2
int ledState = LOW;
int Mode=0;
//Create UDP message object
EthernetUDP Udp;

void setup(){
  Serial.begin(115200); //9600 for a "normal" Arduino board (Uno for example). 115200 for a Teensy ++2 
  Serial.println("OSC test");

  // start the Ethernet connection:
  // NOTE: Alternatively, you can assign a fixed IP to configure your
  //       Ethernet shield.
        Ethernet.begin(mac, ip);   
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    while(true);
  }
  // print your local IP address:
  Serial.print("Arduino IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print("."); 
  }

  Udp.begin(serverPort);
}

void loop(){
  //process received messages
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
    Serial.println("LED on");
  }
  else {
    Serial.println("LED off");
  }

  ledState = !ledState;		 // toggle the state from HIGH to LOW to HIGH to LOW ...

  //send osc message back to controll object in TouchOSC
  //Local feedback is turned off in the TouchOSC interface.
  //The button is turned on in TouchOSC interface whe the conrol receives this message.
  Udp.beginPacket(Udp.remoteIP(), destPort);
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message
}


void MultitoggleMode(OSCMessage &msg, int addrOffset){
  int Mode = msg.getFloat(0);
 // OSCMessage msgOUT(concat("/1/multitoggleMode/ValMode/",char(Mode)));
  OSCMessage msgOUT("/1/multitoggleMode/ValMode/1");
  
  //digitalWrite(ledPin, ledState);
 Serial.println(Mode);

  msgOUT.add(Mode);
//  if (ledState) {
//    Serial.println("LED on");
//  }
//  else {
//    Serial.println("LED off");
//  }

  Mode = !(boolean)Mode;     // toggle the state from HIGH to LOW to HIGH to LOW ...

  //send osc message back to controll object in TouchOSC
  //Local feedback is turned off in the TouchOSC interface.
  //The button is turned on in TouchOSC interface whe the conrol receives this message.
  Udp.beginPacket(Udp.remoteIP(), destPort);
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message
}

void funcValueH(OSCMessage &msg, int addrOffset ){
  int value = msg.getFloat(0);  
  OSCMessage msgOUT_H("/1/faderH/ValueH"); 
  Serial.print("Value = : ");
  Serial.println(value);
  msgOUT_H.add(value);
  Udp.beginPacket(Udp.remoteIP(), destPort);
  msgOUT_H.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_H.empty(); // free space occupied by message
}

void funcValueS(OSCMessage &msg, int addrOffset ){
  int value = msg.getFloat(0);
  OSCMessage msgOUT_S("/1/faderS/ValueS");
  Serial.print("Value = : ");
  Serial.println(value);
  msgOUT_S.add(value);
  Udp.beginPacket(Udp.remoteIP(), destPort);
  msgOUT_S.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_S.empty(); // free space occupied by message
}

void funcValueV(OSCMessage &msg, int addrOffset ){
  int value = msg.getFloat(0); 
  OSCMessage msgOUT_V("/1/faderV/ValueV");  
  Serial.print("Value = : ");
  Serial.println(value);
  msgOUT_V.add(value);
  Udp.beginPacket(Udp.remoteIP(), destPort);
  msgOUT_V.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_V.empty(); // free space occupied by message
}

void funcValueFc(OSCMessage &msg, int addrOffset ){
  float value = msg.getFloat(0); 
  //value=((float)value*(40.0/1023.0))-20.0;
  value=20-((value*40.0)/1023.0);
  OSCMessage msgOUT_Fc("/1/faderFc/ValueFc");  
  Serial.print("Value = : ");
  Serial.println(value);
  msgOUT_Fc.add(value);
  Udp.beginPacket(Udp.remoteIP(), destPort);
  msgOUT_Fc.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc.empty(); // free space occupied by message
}

void funcValueFf(OSCMessage &msg, int addrOffset ){
  float value = msg.getFloat(0); 
  value=(value*(5.0/1023.0))-2.5;
  OSCMessage msgOUT_Ff("/1/faderFf/ValueFf");  
  Serial.print("Value = : ");
  Serial.println(value);
  msgOUT_Ff.add(value);
  Udp.beginPacket(Udp.remoteIP(), destPort);
  msgOUT_Ff.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Ff.empty(); // free space occupied by message
}
 
