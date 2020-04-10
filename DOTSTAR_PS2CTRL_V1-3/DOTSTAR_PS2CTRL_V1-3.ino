#include <Adafruit_DotStar.h>
#include "arduinoFFT.h"
#include <Regexp.h> ///https://github.com/nickgammon/Regexp
#include <FreqMeasureMulti.h> //https://github.com/PaulStoffregen/FreqMeasureMulti

#define NUMPIXELS 72 // Number of LEDs in strip
#define trig 2//camera trigger out
#define OnSwitch 3//switch input pin
#define avg_window 5 // to reduce noise in frequency computation for synth trigger 
#define AudioIn 6 //
#define Refresh_screen 2000 //rate of parameters refreshing sent to the screen in millis
#define compute_delay1 280
#define compute_delay2 300
#define strobefreq0 70.00 //initial strobe freq in mode 1

// Create the strip object
Adafruit_DotStar strip= Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG); 

//create a pulse measurement object based on interupt
FreqMeasureMulti freq1;

//intial parameters
int Se=2;////MODE 0,1 OR 2
float strobefreq=41.00;
float Fshift=0.00;
long phase=0;
long timer=8; //duty cycle (time on)
int incr=1;
int ModeSwitch;
/////color parameters HUE HSV scale
unsigned long hue=2;//color 0 to 65535
int sat=0;// purity of the color  0 to 255 => white light
int val=150;//brightness 0 to 255
uint32_t rgbcolor;
int tog=1;
int previous_val;

//////////////


//////ON/OFF switch variables
int state = HIGH;      // the current state of the output pin
int state2=HIGH;
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
int stro1num = 1;             // ledState used to set the LED strip
long prestro1Millis = 0;        // will store last time LED strip was updated
  // the following variables are long's because the time, measured in miliseconds,
  // will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 1;   // the debounce time, increase if the output flickers
///////////////////////


/////parsing variables
char rxChar;//char variable for serial commands
char rxText[50];
int rxN=0; // Number of chars received
unsigned long count;//for parse function of Serial command. Count the number of axis entered
//////

unsigned long previousMillis = 0; // store previous millis readout (for SInterval)
unsigned long previousMicros = 0; // store previous millis readout (for SInterval)
unsigned long monitortime1=0;
unsigned long monitortime2=0;

uint16_t len=NUMPIXELS;

//PS2 CMD Variables
/////

//Fsynch Variables
long delta_T0;
unsigned long trig_period;
/////
float Fc=0;
float Ff=0;
  
void setup() {
pinMode(trig, OUTPUT);
pinMode(OnSwitch, INPUT);

strip.begin(); //initialize strip
strip.show(); //turn off LEDs within NUMPIXELS

Serial.begin(115200);
Serial1.begin(115200);// serial port to send updated parameters to the screen
Serial2.begin(115200);// Gamepad control in
Serial3.begin(115200);

freq1.begin(AudioIn); //

////set potentiometer for HUE color scale
//hue=
//sat=
//val=
//strobefreq=strobefreq0;
Serial3.print("F0");Serial3.print(strobefreq0);Serial3.println(";");//to tell the light that the controler
                                                                   //is ON such that strobefreq0 is updated in the controller    
Serial3.print("M");Serial3.print(Se);Serial3.println(";");
}

void loop() {
////Serial.println("hello world!");

////set potentiometer for HUE color scale
//hue=
//sat=
//val=


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
   //  Serial.println(rxText);
    
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
      if (val<255) {val=val+1;}
        Serial.print("Light Intensity="); Serial.println(val);
       // Serial1.print("V"); Serial1.println(val);
      }

     ///
     char resultD=ms.Match ("(D)",0);
      if (resultD == REGEXP_MATCHED){
        char captureBuf [4];
        if (val>0) {val=val-1;}
        Serial.print("Light Intensity="); Serial.println(val);
        //Serial1.print("V"); Serial1.println(val);
      }

     ///
     char resultL1=ms.Match ("(L1)",0);
      if (resultL1 == REGEXP_MATCHED){
        char captureBuf [6];
        if (Se==1){
         if (strobefreq<200) {strobefreq=strobefreq+0.05;}
         Serial.print("strobefreq="); Serial.println(strobefreq);
        // Serial1.print("F"); Serial1.println(strobefreq);
//        if (strobefreq>200) {strobefreq=100;}
//        }
        if (strobefreq>1000) {strobefreq=1000;}
       }
        if (Se==2){
         if (Fshift<5) {Fshift=Fshift+0.01;} else {Fshift=5;}
         Serial.print("Fshift="); Serial.println(Fshift);
        // Serial1.print("Fs"); Serial1.println(Fshift);

        }
      }

     ///
     char resultL2=ms.Match ("(L2)",0);
      if (resultL2 == REGEXP_MATCHED){
        char captureBuf [10];
        if (Se==1){
        if (strobefreq>1) {strobefreq=strobefreq-0.05;}
        Serial.print("strobefreq="); Serial.println(strobefreq);
       // Serial1.print("F"); Serial1.println(strobefreq);
       }
        if (Se==2){
         if (Fshift>-5) {Fshift=Fshift-0.01;} else {Fshift=-5;}
         Serial.print("Fshift="); Serial.println(Fshift);
        // Serial1.print("Fs"); Serial1.println(Fshift);
   
        }
      }
    
      char resultSe=ms.Match ("(Se)");
      if (resultSe == REGEXP_MATCHED){
        char captureBuf [6];
        if (Se<2){Se=Se+1;}
        else{Se=0;}
        Serial.print("Se="); Serial.println(Se);
        
        if (Se==1){Serial3.print("F0");Serial3.print(strobefreq0);Serial3.println(";");
        Serial3.print("M");Serial3.print(Se);Serial3.println(";");
        }
        
        if (Se==2){Serial3.print("F0");Serial3.print("0");Serial3.println(";");
        Serial3.print("M");Serial3.print(Se);Serial3.println(";");
        }
      }
        
      //  Serial1.print("M"); Serial1.println(Se);
   }
  
  
   
}    //end if Serial2.available


if (Serial3.available()>0){

  // what we are searching (the target)
  rxChar = Serial3.read();
  
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

      char resultM=ms.Match ("(M)(%d)(;)",0);
      if (resultM == REGEXP_MATCHED){
        char captureBuf [10];
        Se=atoi(ms.GetCapture (captureBuf,1)); 
        if (Se==1){Serial3.print("F0");Serial3.print(strobefreq0);Serial3.println(";");}
        if (Se==2){Serial3.print("F0");Serial3.print("0");Serial3.println(";");}
      }
     

      char resultC1=ms.Match ("(C1;)",0);
      if (resultC1 == REGEXP_MATCHED){
      Serial3.print("F0");Serial3.print(strobefreq0);Serial3.println(";");
      Serial3.print("M");Serial3.print(Se);Serial3.println(";");
      Serial.print("F0");Serial.print(strobefreq0);Serial.println(";");
      }//to tell the light that the controler is on such that strobefreq0 is updated in the controller
    
     ///
      char resultTOF=ms.Match ("(TOF)(%d)(;)",0);
      if (resultTOF == REGEXP_MATCHED){
        char captureBuf [10];
        tog=atoi(ms.GetCapture (captureBuf,1)); 
        if (tog==1){Serial.println("light ON");}
        else {Serial.println("light OFF");}
      }
     
     char resultH=ms.Match ("(H)(%d+)(;)",0);
      if (resultH == REGEXP_MATCHED){
        char captureBuf [10];
        hue=atoi(ms.GetCapture (captureBuf,1))*64; 
        Serial.print("Hue=");Serial.println(hue);
      }

     char resultS=ms.Match ("(S)(%d+)(;)",0);
      if (resultS == REGEXP_MATCHED){
        char captureBuf [10];
        sat=atoi(ms.GetCapture (captureBuf,1)); 
        Serial.print("sat=");Serial.println(sat);
      }
     
     char resultV=ms.Match ("(V)(%d+)(;)",0);
      if (resultV == REGEXP_MATCHED){
        char captureBuf [10];
        val=atoi(ms.GetCapture (captureBuf,1)); 
        Serial.print("val=");Serial.println(val);
      }


     char resultFc=ms.Match ("(Fc)(+?-?%d+.%d+)(;)",0);
      if (resultFc == REGEXP_MATCHED){
        char captureBuf [10];
        Fc=atof(ms.GetCapture (captureBuf,1));
        if (Se==2){Fshift=Fc;Serial.print("Fshift=");Serial.println(Fshift);} 
        if (Se==1){
          //strobefreq=strobefreq0+Fc;
          strobefreq=Fc;
          Serial.print("Fstrobe=");Serial.println(strobefreq);
          }       
        //Serial.print("Fc=");Serial.println(Fc);
      }

      char resultFf=ms.Match ("(Ff)(+?-?%d+.%d+)(;)",0);
      if (resultFf == REGEXP_MATCHED){
        char captureBuf [10];
        Ff=atof(ms.GetCapture (captureBuf,1)); 
        if (Se==2){Fshift=Fc+Ff;
        //Serial.print("Fshift=");Serial.println(Fshift);
        } 
        if (Se==1){
          //strobefreq=strobefreq0+Fc+Ff;Serial.print("Fstrobe=");Serial.println(strobefreq);}           
        strobefreq=Fc+Ff;Serial.print("Fstrobe=");Serial.println(strobefreq);}           
      }
      
    }
}    //end if Serial3.available


if (tog==0){strip.clear();strip.show();} //turn off all LEDs
else {

rgbcolor = strip.gamma32(strip.ColorHSV(hue, sat, val));
//previous_val=val

unsigned long Millis_local;
unsigned long curstro1Millis = millis();  // set time to now
 
 if (Se==0){
          //   kk=0;
             for (int j=0; j<len; j=j+abs(incr)){
              strip.setPixelColor(j,rgbcolor);     
              for (int iii=1; iii<incr; iii++){              
                strip.setPixelColor(j+iii,0);        
                 }       
            }
              strip.show();
              if (millis() - previousMillis >= Refresh_screen){
         // Save timestamp
         previousMillis = millis();
         Serial1.print("F"); Serial1.print(strobefreq);//to remove?
         Serial1.print("Fs"); Serial1.print(Fshift);
         Serial1.print("V"); Serial1.print(val);
         Serial1.print("M"); Serial1.print(Se);Serial1.println(";");
         
         Serial.print("F"); Serial.print(strobefreq);//to remove?
         //Serial.print("Fs"); Serial.print(Fshift);
         Serial.print("V"); Serial.print(val);
         Serial.print("M"); Serial.print(Se);Serial.println(";");
         }
        }
        
 else if (Se==1){

 
      if (micros() - previousMicros >=(unsigned long)(1000000/strobefreq)-compute_delay1) {//implement a delay here for time cost of strip.show()?
          state=HIGH;    
          for (int j=0; j<len; j=j+abs(incr)){              
          strip.setPixelColor(j,rgbcolor);
          for (int iii=1; iii<incr; iii++){              
            strip.setPixelColor(j+iii,0);        
             }       
           }
           strip.show();  
           digitalWrite(trig, HIGH);        
           previousMicros = micros();

      } 
  
      if (micros() - previousMicros >= (unsigned long)(timer*1000)-compute_delay2) {
      previous=state;
      state=LOW;
      }

      
      if (state == LOW && previous == HIGH && micros() - time > debounce) {
        digitalWrite(trig,LOW);
        strip.clear();strip.show();

      }
    if (millis() - previousMillis >= Refresh_screen){
         // Save timestamp
         previousMillis = millis();
         Serial1.print("F"); Serial1.print(strobefreq);//to remove?
         Serial1.print("Fs"); Serial1.print(Fshift);
         Serial1.print("V"); Serial1.print(val);
         Serial1.print("M"); Serial1.print(Se);Serial1.println(";");

         Serial.print("F"); Serial.print(strobefreq);//to remove?
         //Serial.print("Fs"); Serial.print(Fshift);
         Serial.print("V"); Serial.print(val);
         Serial.print("M"); Serial.print(Se);Serial.println(";");

         //Serial3.print("Fs");Serial3.println(Fshift);
         //Serial3.print("F");Serial3.println(strobefreq);

         }  
 }

else if (Se==2){
  


//Serial.print("V"); Serial.println("cool");
if (freq1.available()) {

//trig_period=(100000000/((100000000/(freq1.countToNanoseconds(freq1.read())/1000))+(long)(Fshift*100)));
trig_period=(100000000/((100000000/(freq1.countToNanoseconds(freq1.read())/1000))));
strobefreq=(1000000/((float)trig_period));
if (Fshift!=0.00){
delta_T0=(long)(100000000/((100000000/trig_period)+(long)(Fshift*100)))-trig_period;
} else {delta_T0=0;}
  }
  
  if (Fshift==0.00){delta_T0=0;}
//
//if (micros() - previousMicros >=(unsigned long)((trig_period+delta_T0)+(phase*(trig_period+delta_T0)))) {//implement a delay here for time cost of strip.show()?
//if (micros() - previousMicros >=trig_period+delta_T0) {//implement a delay here for time cost of strip.show()?
if (micros() - previousMicros >=trig_period+delta_T0-compute_delay1) {//implement a delay here for time cost of strip.show()?
  
          delayMicroseconds((unsigned long)(((float)phase/100)*(trig_period+delta_T0)));
          state=HIGH;    
          for (int j=0; j<len; j=j+abs(incr)){              
          strip.setPixelColor(j,rgbcolor);
          for (int iii=1; iii<incr; iii++){              
            strip.setPixelColor(j+iii,0);        
             }       
           }
           strip.show();  
           digitalWrite(trig, HIGH);        
           previousMicros = micros();//
//           Serial.print(" shift=");//Serial.print((float)phase/100);
//           Serial.print((unsigned long)(((float)phase/100)*(trig_period+delta_T0)));           
//           Serial.print(" micros()=");Serial.print(micros());
//           Serial.print(" previousMicros=");Serial.println(previousMicros);
      } 
     // if (micros() - previousMicros >= (unsigned long)((timer*1000)+(phase*(trig_period+delta_T0))-compute_delay2)) {
      if (micros() - previousMicros >= (unsigned long)(timer*1000)-compute_delay2) {
       // delayMicroseconds((unsigned long)(((float)phase/100)*(trig_period+delta_T0)));
      previous=state;
      state=LOW;
      }
      if (state == LOW && previous == HIGH && micros() - time > debounce) {
        digitalWrite(trig,LOW);
        strip.clear();strip.show();
      }     
  if (millis() - previousMillis >= Refresh_screen){
         // Save timestamp
         previousMillis = millis();
         //Serial1.print("F"); Serial1.print(strobefreq);//to remove?
         Serial1.print("F"); Serial1.print(strobefreq);
         Serial1.print("Fs"); Serial1.print(Fshift);
         Serial1.print("V"); Serial1.print(val);
         Serial1.print("M"); Serial1.print(Se);Serial1.println(";");
                  
         Serial.print("F"); Serial.print((1000000/((float)trig_period))-Fshift);
         Serial.print(" trig period="); Serial.print(trig_period);
         Serial.print(" Fs"); Serial.print(Fshift);
         Serial.print("V"); Serial.print(val);
         Serial.print("M"); Serial.print(Se);Serial.println(";");
         Serial.print(" phase=");Serial.print(phase);Serial.print("%");
         Serial.print(" delta_T0=");Serial.print(delta_T0);Serial.println(";");

         //Serial3.print("Fs");Serial3.println(Fshift);
         //Serial3.print("Ft");Serial3.println(strobefreq);

         }

 }

}


}     //////END LOOP
