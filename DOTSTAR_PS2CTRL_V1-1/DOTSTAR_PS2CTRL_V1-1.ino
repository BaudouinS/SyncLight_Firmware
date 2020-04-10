 #include <Adafruit_DotStar.h>
#include "arduinoFFT.h"
#include <Regexp.h> ///https://github.com/nickgammon/Regexp

#define NUMPIXELS 72 // Number of LEDs in strip
#define trig 2//camera trigger out
#define OnSwitch 3//switch input pin


// Create the strip object

//software SPI
//#include <SPI.h>         
//#define DATAPIN    4
//#define CLOCKPIN   5
//Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

Adafruit_DotStar strip= Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG); 
//arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

//intial parameters
float strobefreq=64.44;
float Fshift=0.00;
long timer=10; //duty cycle (time on)
int Light_Int=50;
//volatile uint16_t SpeedVal;
//volatile uint16_t strobeval;
int incr=1;
int ModeSwitch;
/////color parameters HUE HSV scale
unsigned long hue=10000;//color 0 to 65535
int sat=0;// purity of the coglor  0 to 255 => white light
int val=150;//brightness 0 to 255
uint32_t rgbcolor;

//volatile uint16_t Light_Int;
//////////////


//////ON/OFF switch variables
int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
int stro1num = 1;             // ledState used to set the LED strip
long prestro1Millis = 0;        // will store last time LED strip was updated
  // the following variables are long's because the time, measured in miliseconds,
  // will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers
///////////////////////


/////parsing variables
char rxChar;//char variable for serial commands
char rxText[50];
int rxN=0; // Number of chars received
unsigned long count;//for parse function of Serial command. Count the number of axis entered
//////

int SpeedSign;
int Sign;
int StrobeSwitch;
uint32_t color=128*(0x010101);;
//unsigned long previousMillis1 = 0; // store previous millis readout (for SInterval)
unsigned long previousMillis = 0; // store previous millis readout (for SInterval)
unsigned long previous_trig=0;
unsigned long trig_period;
uint16_t len=NUMPIXELS;

//PS2 CMD Variables
int Se=2;
/////

//Fsynch Variables
float Fsynch=40.0;
//unsigned long duration;
/////

/////FUNCTIONS
// uint32_t randColor(){
//    uint32_t color= 0x000000; //GRB
//    
//    uint8_t red=random(0,255);
//    uint8_t green=random(0,255);
//    uint8_t blue=random(0,255);
//  
//    color= color | (green <<16); // green: 0xXX0000
//    color= color | (red <<8); 
//    color= color | blue; 
//    
//    return color;
//    }
    
////////////////////

// uint32_t ColorCode(int Light_Int,int r,int g,int b){//rgb from 0 to 100%
//    uint32_t color= 0x000000; //GRB
//    
////    uint8_t red=r;
////    uint8_t green=g;
////    uint8_t blue=b;
//    uint8_t red=r*Light_Int/100;
//    uint8_t green=g*Light_Int/100;
//    uint8_t blue=b*Light_Int/100;
//    Serial.print("R");Serial.print(red);Serial.print(" G");Serial.print(green);Serial.print(" B");Serial.println(blue);
//
//    color= color | (green <<16); // green: 0xXX0000
//    color= color | (red <<8); 
//    color= color | blue; 
//    //color=(Light_Int)*color;
//    return color;
//    }

//void rising1() {
//  attachInterrupt(0, falling1, RISING);
//  prev_time1 = micros();
//}
//volatile int pwm_value1 = 0;//speed pin 2 //on DUE pin 9
volatile int trig_time = 0;

//ISR Function for AttachInterrupt
void trigSynth(){
    previous_trig=trig_time;

  trig_time = micros();  
    trig_period=trig_time-previous_trig; 

   //  digitalWrite(trig, HIGH);  
  Serial.print("  trig period=");Serial.println(trig_period);

  //Serial.println("Rise Detected");
  //Serial.println(trig_time);
}
  
void setup() {
pinMode(trig, OUTPUT);
pinMode(OnSwitch, INPUT);

strip.begin(); //initialize strip
//strip.clear();
strip.show(); //turn off LEDs within NUMPIXELS
//strip.clear();

Serial.begin(115200);
Serial1.begin(115200);// Freq in - function generator Synch
Serial2.begin(115200);// Gamepad control in

////set potentiometer for HUE color scale
//hue=
//sat=
//val=

}

void loop() {
////set potentiometer for HUE color scale
//hue=
//sat=
//val=

rgbcolor = strip.gamma32(strip.ColorHSV(hue, sat, val));

////ON/OFF switch
//reading = digitalRead(OnSwitch);
//
//  // if the input just went from LOW and HIGH and we've waited long enough
//  // to ignore any noise on the circuit, toggle the output pin and remember
//  // the time
//  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
//    if (state == HIGH){
//      state = LOW;
//      Serial.println("LEDs ON");
//      }
//
//    else {
//      state = HIGH;
//      Serial.println("LEDs OFF");
//      }
//    time = millis();    
//  }
//
////////////////////////
//

//
//
//if (Serial1.available()>0){
//  rxChar = Serial1.read();
//  
//   // Add normal character
//    if(rxChar!='\n'){
//      rxText[rxN]=rxChar;
//      rxN++;
//    }
//    // Text is done -> extract speed values as integers and Print results
//    if(rxChar=='\n') {
//      rxText[rxN]='\0'; // Terminate the string
//      rxN = 0; // Reset the receive counter
// 
//     MatchState ms;
//     ms.Target(rxText) ; 
//     //Serial.println(rxText);
//
//     char resultF=ms.Match ("(F)(.+)");
//      if (resultF == REGEXP_MATCHED){
//        char captureBuf [10];
//        Fsynch=atof(ms.GetCapture (captureBuf,1)); 
//        //Serial.print("Fsynch="); Serial.println(Fsynch);
//        //Serial.println(ms.GetCapture (captureBuf,1));        ; 
//
//      }
//
//     }
//  }


if (Serial2.available()>0){

  // what we are searching (the target)
  rxChar = Serial2.read();
  
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

    
     ///
     char resultRi=ms.Match ("(Ri)",0);
      if (resultRi == REGEXP_MATCHED){
        char captureBuf [10];
         if (incr<NUMPIXELS){incr=incr+1;}
        Serial.print("incr="); Serial.println(incr);

      }


     ///
     char resultLe=ms.Match ("(Le)",0);
      if (resultLe == REGEXP_MATCHED){
        char captureBuf [10];
         if (incr>1){incr=incr-1;}
        Serial.print("incr="); Serial.println(incr);
      }
      
      
      char resultU=ms.Match ("(U)",0);
      if (resultU == REGEXP_MATCHED){
        char captureBuf [10];
      if (Light_Int<255) {Light_Int=Light_Int+1;}
        Serial.print("Light Intensity="); Serial.println(Light_Int);
      }

     ///
     char resultD=ms.Match ("(D)",0);
      if (resultD == REGEXP_MATCHED){
        char captureBuf [4];
        if (Light_Int>0) {Light_Int=Light_Int-1;}
        Serial.print("Light Intensity="); Serial.println(Light_Int);
      }

     ///
     char resultL1=ms.Match ("(L1)",0);
      if (resultL1 == REGEXP_MATCHED){
        char captureBuf [4];
        if (Se==1){
         if (strobefreq<200) {strobefreq=strobefreq+0.1;}
         Serial.print("strobefreq="); Serial.println(strobefreq);
        }
        if (Se==2){
         if (Fshift<5) {Fshift=Fshift+0.02;} else {Fshift=5;}
         Serial.print("Fshift="); Serial.println(Fshift);
         Serial.print("1000000/(Fsynch+Fshift)="); Serial.println((float)1000000/(Fsynch+Fshift));
        }
      }

     ///
     char resultL2=ms.Match ("(L2)",0);
      if (resultL2 == REGEXP_MATCHED){
        char captureBuf [10];
        if (Se==1){
        if (strobefreq>1) {strobefreq=strobefreq-0.1;}
        Serial.print("strobefreq="); Serial.println(strobefreq);
       }
        if (Se==2){
         if (Fshift>-5) {Fshift=Fshift-0.02;} else {Fshift=-5;}
         Serial.print("Fshift="); Serial.println(Fshift);
         Serial.print("1000000/(Fsynch+Fshift)="); Serial.println((float)1000000/(Fsynch+Fshift));
         //float currentMillis = millis();
         //Serial.print("currentMillis - (float)previousMillis");Serial.println(currentMillis - (float)previousMillis);
        }
      }
    

//     char resultSe=ms.Match ("(Se)(%d)",0);
//      if (resultSe == REGEXP_MATCHED){
//        char captureBuf [10];
//        Se=atoi(ms.GetCapture (captureBuf,1)); 
//        Serial.print("Se="); Serial.println(Se);
//      }
      char resultSe=ms.Match ("(Se)",0);
      if (resultSe == REGEXP_MATCHED){
        char captureBuf [4];
        if (Se==2){Se=0;}
        else{Se=Se+1;}
        }
        Serial.print("Se="); Serial.println(Se);
      
      }
  
   
}    //end if Serial2.available



//
//SpeedSign = digitalRead(7);
//Sign=map(SpeedSign,0,1,-1,1);
//ModeSwitch = digitalRead(6);
//StrobeSwitch = digitalRead(4);
//Light_Int=map(pwm_value4,1,1000,0,255);
//incr=map(pwm_value3,1,1000,1,8);
////Serial.println(incr);   
//SpeedVal=map(pwm_value1,1,1000,4,600);
//timer=20000/(SpeedVal);  
//incr=incr*Sign; 
//strobeval=map(pwm_value2,1,1000,2,100);
//
//
//
 //LEDStand(Se,strobefreq,incr,color,len);
//unsigned long CurrentMillis=millis();
//previousMillis=CurrentMillis;

//LEDStand(Se,5,strobefreq,incr,color,len);

//if (state==HIGH) {

//color=Light_Int*(0x010101);

unsigned long Millis_local;
unsigned long curstro1Millis = millis();  // set time to now
 
 if (Se==0){
          //   kk=0;
             for (int j=0; j<len; j=j+abs(incr)){
              strip.setPixelColor(j,color);     
              for (int iii=1; iii<incr; iii++){              
                strip.setPixelColor(j+iii,0);        
                 }       
            }
              strip.show();
             // Serial.println("mode strobe off");
        }
        
 else if (Se==1){

  //unsigned long currentMillis = millis();

      if (millis() - previousMillis >= 1000/strobefreq) {
          // save the last time you turned on the LED
          previousMillis = millis();
          digitalWrite(trig, HIGH);
      }
      
      if (millis() - previousMillis <= timer) {
          //Serial.println(currentMillis - previousMillis);
           for (int j=0; j<len; j=j+abs(incr)){
                //strip.setPixelColor(j,color); 
                //strip.setPixelColor(j,ColorCode(Light_Int,100,50,20));    
                strip.setPixelColor(j,rgbcolor);
                for (int iii=1; iii<incr; iii++){              
                  strip.setPixelColor(j+iii,0);        
                   }       
              }
                strip.show();    
          }
      else{
      
          strip.clear();strip.show();
          digitalWrite(trig, LOW);   
          }
     }

else if (Se==2){

attachInterrupt(4,trigSynth, RISING);

 if (trig_time-previousMillis != 0){
     digitalWrite(trig, HIGH);  
     previousMillis=trig_time; 
     //Serial.println("trigger cam");
 }
unsigned long currentMicros = micros();
//Serial.println(trig_time);
  if (currentMicros - trig_time <= timer*1000){
   
   // strip.fill(color, 0, NUMPIXELS);

 for (int j=0; j<len; j=j+abs(incr)){
                //strip.setPixelColor(j,color); 
                //strip.setPixelColor(j,ColorCode(255,100,100,100));  
                strip.setPixelColor(j,rgbcolor);
                for (int iii=1; iii<incr; iii++){              
                  strip.setPixelColor(j+iii,0);        
                   }       
              }   
  } 

else {
    strip.fill(0x000000, 0, NUMPIXELS);
    digitalWrite(trig, LOW);  

  //Serial.println(currentMicros - trig_time);
  }
  
  strip.show();

}

}     //////END LOOP
