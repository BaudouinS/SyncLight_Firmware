 #include <Adafruit_DotStar.h>
#include "arduinoFFT.h"
#include <Regexp.h> ///https://github.com/nickgammon/Regexp

#define NUMPIXELS 72 // Number of LEDs in strip
#define trig 2//camera trigger out
#define OnSwitch 3//switch input pin


/////for freq measurement with FFT
//#define CHANNEL A1 //freq measurement analog input. teensy 3.6 A1 on pinout 15
//#define SCL_INDEX 0x00
//#define SCL_TIME 0x01
//#define SCL_FREQUENCY 0x02
//#define SCL_PLOT 0x03
/////

// Create the strip object

//software SPI
//#include <SPI.h>         
//#define DATAPIN    4
//#define CLOCKPIN   5
//Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

Adafruit_DotStar strip= Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG); 
//arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

//intial parameters
float strobefreq=60.0;
long timer=5; //duty cycle (time on)
int Light_Int=255;
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

//volatile uint16_t SpeedVal;
//volatile uint16_t strobeval;
int incr=1;
int ModeSwitch;
//volatile uint16_t Light_Int;

int SpeedSign;
int Sign;
int StrobeSwitch;
uint32_t color=128*(0x010101);;
//unsigned long previousMillis1 = 0; // store previous millis readout (for SInterval)
unsigned long previousMillis = 0; // store previous millis readout (for SInterval)

uint16_t len=NUMPIXELS;

//PS2 CMD Variables
int Se=1;
/////

//Fsynch Variables
float Fsynch=40.0;
/////


//////variables declaration for Freq measurement with FFT
//const uint16_t samples = 128//This value MUST ALWAYS be a power of 2
//const double samplingFrequency = 400; //Hz, must be less than 10000 due to ADC
//unsigned int sampling_period_us;
//unsigned long microseconds;
///*
//These are the input and output vectors
//Input vectors receive computed results from FFT
//*/
//double vReal[samples];
//double vImag[samples];
//////////////////////////////////////////////
//

//long stro1interval1 = 50;           // interval at which to blink on(milliseconds)
//long stro1interval2 = 100;           // interval at which to blink off(milliseconds)
//long stro1interval3 = 150;           // interval at which to blink on(milliseconds)
//long stro1interval4 = 1800;           // interval at which to blink off(milliseconds)



//int parsingCMD_num(char rxChar,char CMD){
//
//     MatchState ms;
//     ms.Target(rxChar) ;     
//     char result=ms.Match (CMD,0);
//      if (result == REGEXP_MATCHED){
//        char captureBuf [10];
//        return atoi(ms.GetCapture (captureBuf,1));
//      //  Serial.print("L1= ");Serial.println(L1);
//      }
//
//}

//void Strobe(int timer,float freq,unsigned long Millis){
//
// if (Millis - previousMillis >=1000/freq){// && Millis - previousMillis<((1000/freq)+timer) ){
//   
//   strip.clear();
//   strip.show();
////
////     if (Millis - previousMillis==((1000/freq)+timer)){
////          previousMillis = Millis;
////     }
//   
//   delay(timer);
//   }  
//
//}
//// 
//void LEDStand(int incr,uint32_t color,uint16_t len){
//    
//         //SpeedVal=map(pwm_value,1,1000,4,600);
//         //timer=20000/(SpeedVal);
//         
//         //strobeval=map(pwm_value2,1,1000,4,600);
//         // ModeSwitch = digitalRead(7); 
//          for (int j=0; j<len; j=j+abs(incr)){
//            strip.setPixelColor(j,color);     
//            for (int iii=1; iii<incr; iii++){              
//              strip.setPixelColor(j+iii,0);        
//               }       
//          }
//            strip.show();
//      
//        // Serial.println(1000/strobeval);
//      //Serial.println("LEDStand called");
//}

//void LEDStand(int mode,int timer,int freq,int incr,uint32_t color,uint16_t len){
//       
////int k;
//unsigned long  Millis_local;
//unsigned long  Millis;
//
//        if (mode==0){
//             kk=0;
//             for (int j=0; j<len; j=j+abs(incr)){
//              strip.setPixelColor(j,color);     
//              for (int iii=1; iii<incr; iii++){              
//                strip.setPixelColor(j+iii,0);        
//                 }       
//            }
//              strip.show();
//             // Serial.println("mode strobe off");
//        }
//         else {
//
//
//         if (stro1num == 1 && curstro1Millis - prestro1Millis > stro1interval1) {digitalWrite(stro1Pin, HIGH); stro1num = 2;} // 1st flash on
//
//
//
//
//
//          
//
////             // Serial.println("mode strobe on");
////              Millis=millis();
////             // Serial.print("Millis="); Serial.println(Millis);
////              
////             
////             // Serial.println(kk);
////              
////              if (kk==0){
////                  Millis_local=Millis;
////               //   Serial.print("Millis_local=");Serial.println(Millis_local);
////                 }    
////              
////               if ((long)Millis-(long)Millis_local<=(long)timer){                 
////                   //Serial.print("duty interval ");Serial.println(Millis-Millis_local);
////                   
////                   for (int j=0; j<len; j=j+abs(incr)){
////                       strip.setPixelColor(j,color);     
////                       for (int iii=1; iii<incr; iii++){              
////                           strip.setPixelColor(j+iii,0);        
////                       }       
////                    }
////                    
////                   strip.show();
////                   kk=1;
////                   }
////              
////               else {strip.clear();
////                   strip.show();
////                   // Serial.println("Off Interval");
////                   }
////  
////              if ((long)Millis-(long)Millis_local>=(long)1000.0/(long)freq){
////              kk=0;
////              }         
////            
////            //Serial.print("(float)Millis-(float)Millis_local=");Serial.print((float)Millis-(float)Millis_local);Serial.print(" at period (ms) ");Serial.println(1000.0/freq);
//}
//
//}
        
///////////////////////
 uint32_t randColor(){
    uint32_t color= 0x000000; //GRB
    
    uint8_t red=random(0,255);
    uint8_t green=random(0,255);
    uint8_t blue=random(0,255);
  
    color= color | (green <<16); // green: 0xXX0000
    color= color | (red <<8); 
    color= color | blue; 
    
    return color;
    }
    
////////////////////
 uint32_t ColorCode(int Light_Int,int r,int g,int b){//grb from 0 to 25
    uint32_t color= 0x000000; //GRB
    
    uint8_t red=r;
    uint8_t green=g;
    uint8_t blue=b;
  
    color= color | (green <<16); // green: 0xXX0000
    color= color | (red <<8); 
    color= color | blue; 
    color=(Light_Int/10)*color;
    return color;
    }


  
void setup() {
pinMode(trig, OUTPUT);
pinMode(OnSwitch, INPUT);

strip.begin(); //initialize strip
//strip.clear();
strip.show(); //turn off LEDs within NUMPIXELS
//strip.clear();

Serial.begin(115200);
//Serial1.begin(115200);// Freq in - function generator Synch
Serial2.begin(115200);// Gamepad control in


//while (Serial2.available()==0)
//{
//  }
//
//LEDStand(1,Light_Int*(0x010101),len);
  
//Serial.println("ready to check PS2 pad connection.");
//delay(5000);
//
//
//
//
//
//
//Serial.println("If nothing detected, reset boards.");
//////put message for connection with PS2 enable


}

void loop() {

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
        char captureBuf [10];
        if (Light_Int>0) {Light_Int=Light_Int-1;}
        Serial.print("Light Intensity="); Serial.println(Light_Int);
      }

     ///
     char resultL1=ms.Match ("(L1)",0);
      if (resultL1 == REGEXP_MATCHED){
        char captureBuf [10];
        if (strobefreq<200) {strobefreq=strobefreq+0.1;}
        Serial.print("strobefreq="); Serial.println(strobefreq);
      }


     ///
     char resultL2=ms.Match ("(L2)",0);
      if (resultL2 == REGEXP_MATCHED){
        char captureBuf [10];
        if (strobefreq>1) {strobefreq=strobefreq-0.1;}
        Serial.print("strobefreq="); Serial.println(strobefreq);
      }
    

     char resultSe=ms.Match ("(Se)(%d)",0);
      if (resultSe == REGEXP_MATCHED){
        char captureBuf [10];
        Se=atoi(ms.GetCapture (captureBuf,1)); 
        Serial.print("Se="); Serial.println(Se);
      }
    
  }
  
   
}//end if Serial2.available



//
//SpeedSign = digitalRead(7);
//Sign=map(SpeedSign,0,1,-1,1);
//ModeSwitch = digitalRead(6);
//StrobeSwitch = digitalRead(4);
//Light_Int=map(pwm_value4,1,1000,0,255);
color=Light_Int*(0x010101);
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
 else {

  //unsigned long currentMillis = millis();

      if (millis() - previousMillis >= 1000/strobefreq) {
          // save the last time you turned on the LED
          previousMillis = millis();
          digitalWrite(trig, HIGH);
      }
      
      if (millis() - previousMillis <= timer) {
          //Serial.println(currentMillis - previousMillis);
           for (int j=0; j<len; j=j+abs(incr)){
                strip.setPixelColor(j,color);     
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
 // }
 // else{strip.clear();strip.show();}
 // previous = reading;      
}
