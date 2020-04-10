#include <Adafruit_DotStar.h>
#include <Regexp.h> ///https://github.com/nickgammon/Regexp

#define NUMPIXELS 72 // Number of LEDs in strip
#define trigger 2//trigger out
// Create the strip object

//software SPI
//#include <SPI.h>         
//#define DATAPIN    4
//#define CLOCKPIN   5
//Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

Adafruit_DotStar strip= Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG); 

////////PARAMETERS////////////////////
long strobefreq=120;
long timer=5; //duty cycle (time on)
///////////////////////////
int Fader1; 
int Fader2; 
int Fader3; 
int Fader4;

int Light_Int=0;
int RED;
int GREEN;
int BLUE;
 
char rxChar;//char variable for serial commands
char rxText[50];
int rxN=0; // Number of chars received
unsigned long count;//for parse function of Serial command. Count the number of axis entered

//volatile uint16_t SpeedVal;
//volatile uint16_t strobeval;
int incr=1;
int ModeSwitch;
//volatile uint16_t Light_Int;

//float timer;//=50;
int SpeedSign;
int Sign;
int StrobeSwitch;
uint32_t color=128*(0x010101);;
//unsigned long previousMillis1 = 0; // store previous millis readout (for SInterval)
unsigned long previousMillis = 0; // store previous millis readout (for SInterval)

uint16_t len=NUMPIXELS;

//PS2 CMD Variables
int Se=0;
int k=0;


int stro1num = 1;             // ledState used to set the LED strip
long prestro1Millis = 0;        // will store last time LED strip was updated

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
    //color=(Light_Int/10)*color;
    color=(Light_Int/10)*color;

    return color;
    }


  
void setup() {

pinMode(trigger, OUTPUT);

strip.begin(); //initialize strip
//strip.clear();
strip.show(); //turn off LEDs within NUMPIXELS
//strip.clear();

Serial.begin(115200);
Serial2.begin(115200);


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
        if (strobefreq<400) {strobefreq=strobefreq+1;}
        Serial.print("strobefreq="); Serial.println(strobefreq);
      }


     ///
     char resultL2=ms.Match ("(L2)",0);
      if (resultL2 == REGEXP_MATCHED){
        char captureBuf [10];
        if (strobefreq>1) {strobefreq=strobefreq-1;}
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

//Fader1 = analogRead(A0);//RED
//Fader2 = analogRead(A1);//GREEN
//Fader3 = analogRead(A2);//BLUE

//Fader4 = analogRead(A0);//INTENSITY
//Light_Int=map(Fader4,0,1023,255,0);


//
//RED=map(Fader1,0,1023,0,255);
//GREEN=map(Fader2,0,1023,0,255);
//BLUE=map(Fader3,0,1023,0,255);

//Light_Int=map(Fader4,0,1023,0,255);

//
//
//color=ColorCode(Light_Int,RED,GREEN,BLUE);
//
//Serial.print("RED:");Serial.print(RED);
//Serial.print("GREEN:");Serial.print(GREEN);
//Serial.print("BLUE:");Serial.print(BLUE);
//Serial.print("Light_Int:");Serial.println(Light_Int);

color=Light_Int*0x010101;

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
 //LEDStand(Se,strobefreq,incr,color,len);
//unsigned long CurrentMillis=millis();
//previousMillis=CurrentMillis;
//LEDStand(Se,5,strobefreq,incr,color,len);

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
 

 // unsigned long currentMillis = millis();

//  if (currentMillis - previousMillis >= 1000/strobefreq) {
    // save the last time you blinked the LED
//    previousMillis = currentMillis;
//  }
//
 // if (currentMillis - previousMillis <= timer) {
         for (int j=0; j<len; j=j+abs(incr)){
              strip.setPixelColor(j,color);     
              for (int iii=1; iii<incr; iii++){              
                strip.setPixelColor(j+iii,0);        
                 }       
            }
              strip.show();

            delay(timer);
            digitalWrite(trigger, LOW);
   
            strip.clear();
            strip.show();
            delay(1000/strobefreq);////strobe freq  
             // Serial.println("mode strobe off");

            digitalWrite(trigger, HIGH);
  //      else {strip.clear();strip.show();}
  }
        
}
