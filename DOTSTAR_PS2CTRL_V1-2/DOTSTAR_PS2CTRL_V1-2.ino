/**
 * DOTSTAR_PS2CTRL. 
 *
 * Control LED strip DOTSTAR (ADAFRUIT) with a PS2 Gamepad
 * 
 * Mode Se=0 No strobe
 * Mode Se=1 Manual strobe
 * Mode Se=2 Strobe triggered with function generator (synthesizer) with phase and frequency shift parameter
 * 
 * Controller: Teensy 3.6
 * 
 * By Baudouin Saintyves 8/10/2019
 */


#include <Adafruit_DotStar.h>
#include "arduinoFFT.h"
#include <Regexp.h> ///https://github.com/nickgammon/Regexp

#define NUMPIXELS 72 // Number of LEDs in strip
#define trig 2//camera trigger out
#define OnSwitch 3//switch input pin
#define AudioIn 4//synth audio signal input
#define avg_window 5 // to reduce noise in frequency computation for synth trigger 
#define Refresh_screen 200 //rate of parameters refreshing sent to the screen in millis

// Create the strip object

Adafruit_DotStar strip= Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG); 

/////////////VARIABLES////////////////////////////

//////INITIAL PARAMETERS/////////////
float strobefreq=60.00; //for mode 1
long timer=5; //duty cycle (time on)
int incr=1;
//strobe trig
volatile int trig_time = 0;
float Fshift=0.00;   ////SET UP a control fader
int phase=0;        // phase shift From 0 to 100% of the measured period time (setup periodic boundaries)////SET UP a control Knob = use big knob cover for playability? use digital encoder for full revolution
//MODE SELECTION
int Se=2;////PS2 CMD Variables
/////
//color parameters HUE HSV scale
unsigned long hue=2;//color 0 to 65535
int sat=0;// purity of the color  0 to 255 => white light
int val=150;//brightness 0 to 255
uint32_t rgbcolor;
/////
/////////////////////////////////////

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


////Variable for LED Move (not implemented yet)
int SpeedSign;
int Sign;
int StrobeSwitch;
//uint32_t color=128*(0x010101);;
//unsigned long previousMillis1 = 0; // store previous millis readout (for SInterval)
unsigned long previousMillis = 0; // store previous millis readout (for SInterval)
unsigned long previousMicros = 0; // store previous millis readout (for SInterval)

//Fsynch Variables
unsigned long trig_period;
unsigned long delta_time=0;
unsigned long previous_trig=0;
//unsigned long previousMicros = 0; // store previous millis readout (for SInterval)
//unsigned long previous_trig_period=0;
unsigned long Nshift=-1;
float Fsynch=0.00;
unsigned long trig_period_avg[avg_window];//shifting average
int N_avg=-1;
unsigned long previous_trig_period;
/////


//LED SETUP VARIABLES
uint16_t len=NUMPIXELS;

/////////////FUNCTIONS////////////
float average (unsigned long * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

//ISR Function for AttachInterrupt
void trigSynth(){ 
  previous_trig=trig_time;
  previous_trig_period=trig_period_avg[N_avg];
  trig_time = micros(); 
  
  if (N_avg<avg_window){N_avg=N_avg+1;}else{N_avg=0;}
  trig_period_avg[N_avg]=trig_time-previous_trig; 
  trig_period=average (trig_period_avg,avg_window);//trig_time-previous_trig;

  //trig_period=trig_time-previous_trig;


//Serial.println(previous_trig_period-trig_time+previous_trig<100);
    
//  if (previous_trig_period-trig_time+previous_trig<100){  
//         trig_period=average (trig_period_avg,avg_window);//trig_time-previous_trig;
//  }
//
//  else{
//    for (int i = 0 ; i < avg_window  ; i++){
//      trig_period_avg[i]=trig_time-previous_trig; 
//      }
//    trig_period=trig_time-previous_trig;
//    }
    

  //trig_period=trig_time-previous_trig; 
  Fsynch=(100000000/trig_period)+(long)(Fshift*100);
  
  //digitalWrite(trig, HIGH);  
  //Serial.println("Rise Detected");
  //Serial.println(trig_time);
  //Serial.print("F measured=");Serial.print(100000000/trig_period);
  //Serial.print("  Fsynch=");Serial.print((unsigned long)Fsynch);
  //Serial.print("  period=");Serial.print(100000000/(long)Fsynch);
  //Serial.print("  trig period=");Serial.println(trig_period);
  //Serial.print("  delta_T0=");Serial.println((100000000/(long)Fsynch)-trig_period);

}
  
void setup() {
pinMode(trig, OUTPUT);
pinMode(OnSwitch, INPUT);

strip.begin(); //initialize strip
strip.show(); //turn off LEDs within NUMPIXELS

Serial.begin(115200);
Serial1.begin(115200);// serial port to send updated parameters to the screen
Serial2.begin(115200);// Gamepad control in



////set potentiometer for HUE color scale
//hue=
//sat=
//val=

}

void loop() {


////
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
     Serial.println(rxText);
    
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
         if (strobefreq<200) {strobefreq=strobefreq+0.02;}
         Serial.print("strobefreq="); Serial.println(strobefreq);
        // Serial1.print("F"); Serial1.println(strobefreq);
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
        if (strobefreq>1) {strobefreq=strobefreq-0.02;}
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
        }
      //  Serial1.print("M"); Serial1.println(Se);
      }
  
   
}    //end if Serial2.available



unsigned long Millis_local;
unsigned long curstro1Millis = millis();  // set time to now


 if (Se==0){
  // attachInterrupt(AudioIn,trigSynth, RISING);

          //   kk=0;
             for (int j=0; j<len; j=j+abs(incr)){
              strip.setPixelColor(j,rgbcolor);
              for (int iii=1; iii<incr; iii++){              
                strip.setPixelColor(j+iii,0);        
                 }       
            }
              strip.show();
             // Serial.println("mode strobe off");
              if (millis() - previousMillis >= Refresh_screen){
         // Save timestamp
         previousMillis = millis();
         Serial1.print("F"); Serial1.print(strobefreq);//to remove?
         //Serial1.print("Fs"); Serial1.print(Fshift);
         Serial1.print("V"); Serial1.print(val);
         Serial1.print("M"); Serial1.print(Se);
         Serial1.print("Ft");Serial1.println((float)(Fsynch/100));

         Serial.print("F");Serial.print(strobefreq);
         //Serial.print("Fs"); Serial.print(Fshift);
         Serial.print("V"); Serial.print(val);
         Serial.print("Ft");Serial.print((float)(Fsynch/100));
         Serial.print("M"); Serial.println(Se);

         }
        }
        
 else if (Se==1){
        
    // attachInterrupt(AudioIn,trigSynth, RISING);

  //unsigned long currentMillis = millis();

      if (micros() - previousMicros >= (unsigned long)(1000000/strobefreq)) {

          // save the last time you turned on the LED
          previousMicros = micros();
          digitalWrite(trig, HIGH);
         // Serial.print("Freq=");Serial.print(strobefreq);Serial.print(" Period=");Serial.println(100000/(unsigned long)(strobefreq*100));
      }
      
      if (micros() - previousMicros <= timer*1000) {
          //Serial.println(currentMillis - previousMillis);
           for (int j=0; j<len; j=j+abs(incr)){
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
     
         if (millis() - previousMillis >= Refresh_screen){
         // Save timestamp
         previousMillis = millis();
         Serial1.print("F"); Serial1.print(strobefreq);//to remove?
         Serial1.print("Ft");Serial1.print((float)(Fsynch/100));

         //Serial1.print("Fs"); Serial1.print(Fshift);
         Serial1.print("V"); Serial1.print(val);
         Serial1.print("M"); Serial1.println(Se);
         //Serial1.print("Ft");Serial1.println((float)(Fsynch/10));
         
         //Serial.print("Fs"); Serial.print(Fshift);
         Serial.print("V"); Serial.print(val);
         Serial.print("M"); Serial.print(Se);
         Serial.print("F");Serial.print(strobefreq);
         Serial.print("Ft");Serial.print((float)(Fsynch/100));
         Serial.print(" Period=");Serial.println((unsigned long)(1000000/strobefreq));
         }
     
     }

else if (Se==2){

attachInterrupt(AudioIn,trigSynth, RISING);
//Serial.print("period=");Serial.println(1000000/Fsynch);
 
 

 if (micros() - previousMicros >= 100000000/(unsigned long)Fsynch) {
       previousMicros = micros();
       digitalWrite(trig, HIGH); //trigger camera
}

 //if (micros() - previousMicros <= timer*1000){
 if (micros() - trig_time >= phase*trig_period/100 && micros() - trig_time <= timer*1000+(phase*trig_period/100)){
       //Serial.print("phase=");Serial.print(phase);Serial.print(" trig period=");Serial.print(trig_period);Serial.print("  period shift=");Serial.println(phase*trig_period/100);
       for (int j=0; j<len; j=j+abs(incr)){                
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
         
//         if (millis() - previousMillis >= Refresh_screen){
//         // Save timestamp
//         previousMillis = millis();
//       //  Serial1.print("F"); Serial1.print(strobefreq);//to remove?
//         Serial1.print("Fs"); Serial1.print(Fshift);
//         Serial1.print("V"); Serial1.print(val);
//         Serial1.print("M"); Serial1.print(Se);
//         Serial1.print("Ft");Serial1.println((float)(Fsynch/100));
//
////         Serial.print("Fs"); Serial.print(Fshift);
////         Serial.print("V"); Serial.print(val);
////         Serial.print("M"); Serial.print(Se);
////         Serial.print("Ft");Serial.println((float)(Fsynch/100));
//
//         }
}

//
// if (trig_time-previous_trig != 0){
//
//       digitalWrite(trig, HIGH); //trigger camera
//       Nshift=Nshift+1;
//       trig_period=trig_time-previous_trig;
//       delta_time=((unsigned long)( (Nshift+1)*(100000000/((100000000/trig_period)+(long)(Fshift*100)))-trig_period) ) % trig_period;
//       //delta_time=((Nshift+1)*delta_T0) % trig_period;
//       //Serial.println((unsigned long)(100000000/trig_period));
////       Serial.print("Fshift=");Serial.print(Fshift);
////       
////       Serial.print("  measured Freq=");Serial.print((unsigned long)(100000000/trig_period));
////       
////       Serial.print("  shifted frequence=");Serial.print((100000000/trig_period)+(long)(Fshift*100));
////
////       Serial.print("  trig_period=");Serial.print(trig_period);
////       
////       Serial.print("  shifted period=");Serial.print((100000000/((100000000/trig_period)+(long)(Fshift*100))));
////       
////       Serial.print("  delta_T0=");Serial.print((100000000/((100000000/trig_period)+(long)(Fshift*100)))-trig_period);
////       
////       Serial.print("  Nshift=");Serial.print(Nshift);
////       
////       Serial.print("  delta_time=");Serial.println(delta_time);
////       //if (delta_time>trig_period-5 && delta_time<trig_period+5){Nshift=-1;} //Nshift reinitialised at 0 when delta_time = trigger time duration (period) with a 2*5us error range 
//       previous_trig=trig_time; 
//
// }
//delta_time=0;
//  //Serial.println(delta_time);
// unsigned long currentMicros = micros();
////Serial.println(trig_time);
//  //if (currentMicros - trig_time <= timer*1000){
//  
//   if (currentMicros - trig_time >= delta_time && currentMicros - trig_time <= timer*1000+delta_time){
//    
//      for (int j=0; j<len; j=j+abs(incr)){                
//                strip.setPixelColor(j,rgbcolor);
//                for (int iii=1; iii<incr; iii++){              
//                  strip.setPixelColor(j+iii,0);        
//                   }       
//              }   
//  } 
//
//else {
//    strip.fill(0x000000, 0, NUMPIXELS);
//    digitalWrite(trig, LOW);  
//
//  //Serial.println(currentMicros - trig_time);
//  }
//  
//  strip.show();
//
//}

}     //////END LOOP
