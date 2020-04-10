#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Regexp.h> ///https://github.com/nickgammon/Regexp

// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
 Adafruit_PCD8544 display = Adafruit_PCD8544(5, 4, 3);
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!


#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

const int BacklightPin = 9;   
float F;
float Fs;
float Ft;
int M;
int Val;
int Phase;
unsigned long previousMillis=0;

/////parsing variables
char rxChar;//char variable for serial commands
char rxText[50];
int rxN=0; // Number of chars received
unsigned long count;//for parse function of Serial command. Count the number of axis entered
//////


void setup() {
 
 pinMode(BacklightPin, OUTPUT);      
 
 Serial.begin(115200);
 Serial1.begin(115200);

  display.begin();
  // init done

  // you can change the contrast around to adapt the display
  // for the best viewing!
  display.setContrast(50);
  display.display(); // show splashscreen
  //delay(2000);
  display.clearDisplay();   // clears the screen and buffer
  digitalWrite(BacklightPin, HIGH);  
}

void loop() {
  
if (Serial1.available()>0){

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
     Serial.println(rxText);

//     char resultFt=ms.Match ("(Ft)(%d+.%d+) (;)",0);
//      if (resultFt == REGEXP_MATCHED){
//        char captureBuf [10];
//        Ft=atof(ms.GetCapture (captureBuf,1)); 
//        //Serial.print("Ft="); Serial.println(Ft);
//        }

     char resultV=ms.Match ("(V)(%d+)(;)",0);
      if (resultV == REGEXP_MATCHED){
        char captureBuf1 [10];
        Val=atoi(ms.GetCapture (captureBuf1,1)); 
        //Serial.print("V="); Serial.print(Val);
        }

    char resultFs=ms.Match ("(Fs)(%-?%+?%d+.%d+)(;)",0);
      if (resultFs == REGEXP_MATCHED){
        char captureBuf2 [10];
        Fs=atof(ms.GetCapture (captureBuf2,1)); 
        //Serial.print("Fs="); Serial.print(Fs);
        }
        
    char resultF=ms.Match ("(F)(%d+.%d+)(;)",0);
      if (resultF == REGEXP_MATCHED){
        char captureBuf3 [10];
        F=atof(ms.GetCapture (captureBuf3,1)); 
        //Serial.print("F="); Serial.println(F); 
        }

        char resultM=ms.Match ("(M)(%d)(;)",0);
          if (resultM == REGEXP_MATCHED){
        char captureBuf4 [10];
        M=atoi(ms.GetCapture (captureBuf4,1)); 
        //Serial.print("M="); Serial.println(M);
        }

     }
     //delay(10);
   }  ////END SERIAL1.AVAILABLE
  


//  display.setTextSize(1.5);
//  display.setCursor(1,1);
//  //display.setTextColor(WHITE, BLACK); // 'inverted' text
//  display.setTextColor(BLACK); // 'inverted' text
//
//  display.print("MODE ");display.println(M);
//  display.display();
//
//  // text display tests
//  display.setTextSize(1.5);
//  //display.setTextColor(BLACK);
//  display.setCursor(1,10);
//  //display.println("Hello, world!");
//  //display.setTextColor(BLACK); // 'inverted' text
//  //display.println(3.141592);
//  display.print("F:");display.print(F);display.println(" Hz");
//  
//  //display.setTextSize(2);
//  //display.setTextColor(BLACK);
//  //display.print("0x"); display.println(0xDEADBEEF, HEX);
//  display.display();
//  //delay(2000);
  
   if (millis() - previousMillis >= 1000){
         // Save timestamp
         previousMillis = millis();
         
         
  switch (M) {

  case 0:
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1);
  display.setCursor(1,1);  
  display.setTextColor(BLACK); // 'inverted' text
  display.print("MODE ");display.println(M);
  display.display();

  display.setTextSize(1);
  display.setCursor(40,1);  
  display.setTextColor(BLACK); // 'inverted' text
  display.print("Int ");display.println(Val);
  display.display();

  display.setTextSize(2);
  display.setCursor(0,10); 
  //display.print("F:");
  display.print(F);
  display.display();
  
  display.setTextSize(1);
  display.setCursor(72,17); 
  display.println("Hz");
  display.display();
  break;

  case 1:
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1);
  display.setCursor(1,1);  
  display.setTextColor(BLACK); // 'inverted' text
  display.print("MODE ");display.println(M);
  display.display();

  display.setTextSize(1);
  display.setCursor(40,1);  
  display.setTextColor(BLACK); // 'inverted' text
  display.print("Int ");display.println(Val);
  display.display();

  display.setTextSize(2);
  display.setCursor(0,10); 
  //display.print("F:");
  display.print(F);
  display.display();

  display.setTextSize(1);
  display.setCursor(72,17); 
  display.println("Hz");
  display.display();
  break;

 
  case 2:
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1);
  display.setCursor(1,1);  
  display.setTextColor(BLACK); // 'inverted' text
  display.print("MODE ");display.println(M);
  display.display();

  display.setTextSize(1);
  display.setCursor(40,1);  
  display.setTextColor(BLACK); // 'inverted' text
  display.print("Int ");display.println(Val);
  display.display();

  display.setTextSize(2);
  display.setCursor(0,10); 
  //display.print("F:");
  display.print(F);//display.println("Hz");
  display.display();
  
  display.setTextSize(2);
  display.setCursor(0,30); 
  if (Fs>=0){display.print("+");display.print(Fs);}
  else{display.print(Fs);}
  display.display();
  
  display.setTextSize(1);
  display.setCursor(72,37); 
  display.println("Hz");
  display.display();

   break;
   }
  
   }
   
//delay(100);
  
  
}
