/*
  WiFiAccessPoint.ino creates a WiFi access point and provides a web server on it.

  Steps:
  1. Connect to the access point "yourAp"
  2. Point your web browser to http://192.168.4.1/H to turn the LED on or http://192.168.4.1/L to turn it off
     OR
     Run raw TCP "GET /H" and "GET /L" on PuTTY terminal with 192.168.4.1 as IP address and 80 as port

  Created for arduino-esp32 on 04 July, 2018
  by Elochukwu Ifediora (fedy0)
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>


#define LED_BUILTIN 13   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

// Set these to your desired credentials.
const char *ssid = "StrobeWifi";
const char *password = "Shapes2019";

//WiFiServer server(80);
WiFiUDP Udp;
WiFiServer server(80);

const IPAddress outIp(10,40,10,105);        // remote IP (not needed for receive)
const unsigned int outPort = 9000;          // remote port (not needed for receive)
const unsigned int localPort = 8000;        // local port to listen for UDP packets (here's where we send the packets)

unsigned int ledState = LOW;              // LOW means led is *on*

void setup() {
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  //Serial.println();
  //Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  //WiFi.softAP(ssid, password);
  //IPAddress myIP = WiFi.softAPIP();
  //Serial.print("AP IP address: ");
  //Serial.println(myIP);
  //server.begin();

  //Serial.println("Server started");

//  Udp.begin(localPort);
// Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

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


}

void loop() {

    OSCMsgReceive();

//  WiFiClient client = server.available();   // listen for incoming clients
//
//  if (client) {                             // if you get a client,
//    Serial.println("New Client.");           // print a message out the serial port
//    String currentLine = "";                // make a String to hold incoming data from the client
//    while (client.connected()) {            // loop while the client's connected
//      if (client.available()) {             // if there's bytes to read from the client,
//        char c = client.read();             // read a byte, then
//        Serial.write(c);                    // print it out the serial monitor
//        if (c == '\n') {                    // if the byte is a newline character
//
//          // if the current line is blank, you got two newline characters in a row.
//          // that's the end of the client HTTP request, so send a response:
//          if (currentLine.length() == 0) {
//            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
//            // and a content-type so the client knows what's coming, then a blank line:
//            client.println("HTTP/1.1 200 OK");
//            client.println("Content-type:text/html");
//            client.println();
//
//            // the content of the HTTP response follows the header:
//            client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
//            client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");
//
//            // The HTTP response ends with another blank line:
//            client.println();
//            // break out of the while loop:
//            break;
//          } else {    // if you got a newline, then clear currentLine:
//            currentLine = "";
//          }
//        } else if (c != '\r') {  // if you got anything else but a carriage return character,
//          currentLine += c;      // add it to the end of the currentLine
//        }
//
//        // Check to see if the client request was "GET /H" or "GET /L":
//        if (currentLine.endsWith("GET /H")) {
//          digitalWrite(LED_BUILTIN, HIGH);               // GET /H turns the LED on
//        }
//        if (currentLine.endsWith("GET /L")) {
//          digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off
//        }
//      }
//    }
//    // close the connection:
//    client.stop();
//    Serial.println("Client Disconnected.");
//  }
}

void OSCMsgReceive(){
  OSCMessage msgIN;
  int size;
  if((size = Udp.parsePacket())>0){
    while(size--)
      msgIN.fill(Udp.read());
    if(!msgIN.hasError()){
      msgIN.route("/OnOff/toggle1",toggleOnOff);
      msgIN.route("/Fader/Value",funcValue);
    }
  }
}

void toggleOnOff(OSCMessage &msg, int addrOffset){
  ledState = (boolean) msg.getFloat(0);
  OSCMessage msgOUT("/OnOff/toggle1");

  //digitalWrite(ledPin, ledState);

  msgOUT.add(ledState);
  if (ledState) {
    Serial.println("LED on");
  }
  else {
    Serial.println("LED off");
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

void funcValue(OSCMessage &msg, int addrOffset ){

  int value = msg.getFloat(0);
  OSCMessage msgOUT("/Fader/Value");

  Serial.print("Value = : ");
  Serial.println(value);

  msgOUT.add(value);

  Udp.beginPacket(Udp.remoteIP(), outPort);
  msgOUT.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT.empty(); // free space occupied by message
}
