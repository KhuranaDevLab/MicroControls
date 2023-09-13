//Code developed by Gulshan Khurana, to use rotary encoder for data input management

  //Libraries
    #include <Wire.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SH110X.h> //for OLED display
    #include <ezButton.h>
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>

  //MPU6050
    Adafruit_MPU6050 mpu;
    uint16_t MPU_addr = 0x68;
    size_t Size = 6;
    bool yes = true;
    int8_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    int minVal=265;
    int maxVal=402;
    double z;
    int Z3;

  // Rotary Encoder Inputs
    const byte CLK = 12;//B00010000;// 2; //using external hardware interrupt
    const byte DT = 16;//B00100000; //3; // do not change these defined pins
    const byte SW = 27;//B00100000;// 4;

    unsigned int X1 = 0;
    unsigned int X2 = 0;
    bool Fast = false;
    int Speed;
    unsigned int Distance = 0;
    unsigned int NoC = 0; //Number of Cycles
    unsigned int CC = 0; //Cycles completed
    bool MOVE = false;
    bool START = false;
    int currentStateCLK;
    int lastStateCLK = 0;
    String currentDir ="";
    unsigned long lastButtonPress = 0;
    int Menu = 0;

  //OLED parameters//
    #define i2c_Address 0x3c 
    #define SCREEN_WIDTH 128 
    #define SCREEN_HEIGHT 64 
    #define OLED_RESET -1  
    uint32_t preclk = 400000;
    uint32_t postclk = 100000;
    Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



    #define NUMFLAKES 10
    #define XPOS 0
    #define YPOS 1
    #define DELTAY 2

    #define LOGO16_GLCD_HEIGHT 16
    #define LOGO16_GLCD_WIDTH  16

    //OLED 0.9inch
    //SDA -> pin 20.
    //SCK -> I2C SCl.

    //Limit Switch 
      ezButton limitSwitch(7);// for arduino 7, for esp32 18
      int test;
      int LIMIT = 7;
      int state;

    //Stepper
      const int dirPin = 22;
      const int stepPin = 24;
      const int MS0 = 40;
      const int MS1 = 42;
      const int stepsPerRevolution = 12800;
      bool isStopped = false;
      #define MAX_POSITION 0x7FFFFFFF

//fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
void setup() {
  Serial.begin(115200);
  display.begin(i2c_Address, true);
  delay(1000);

  //MPU6050 communication
      Wire.begin();                      // Initialize comunication
      Wire.beginTransmission(MPU_addr);       // Start communication with MPU6050 // MPU=0x68
      Wire.write(0x6B);                  // Talk to the register 6B
      Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
      Wire.endTransmission(true);  

  //Display DEVELOPED BY GULSHAN KHURANA for 3 sec
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(18,16);
    display.println("Developed by");
    display.setCursor(12,32);
    display.println("Gulshan Khurana");
    display.display();
    delay(3000);

  //If serial has not started then wait for 2 sec
    while (!Serial) {
        delay(2000);
      }

  //INPUTS
    pinMode(CLK,INPUT);
    pinMode(DT,INPUT);
    pinMode(SW, INPUT_PULLUP);
    lastStateCLK = digitalRead(CLK);
    attachInterrupt(digitalPinToInterrupt(CLK), handleEncoder, CHANGE);

  //Limit Switch
    limitSwitch.setDebounceTime(1);
    pinMode(7,INPUT_PULLUP); //for arduino 7, for esp32 18

  //Stepper HOme
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(MS0, OUTPUT);
    pinMode(MS1, OUTPUT);
    digitalWrite(MS0, LOW); 
    digitalWrite(MS1, LOW);
    state = limitSwitch.getState();
    limitSwitch.loop();
    while(state == HIGH) {
      digitalWrite(dirPin, LOW);
      digitalWrite(stepPin, HIGH);
      delay(50);
      digitalWrite(stepPin, LOW); 
      delay(50);
      state = digitalRead(7);
      Serial.println("HIGH");
    }
    if(state == LOW){
      digitalWrite(dirPin, HIGH);
      for(int x = 0; x < 600; x++)
	      {
        digitalWrite(stepPin, HIGH);
        delay(50);
        digitalWrite(stepPin, LOW); 
        delay(50);
        Serial.println("low");
        }
    } 
 }
//fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
void loop() {

  //MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,Size,yes);

  //Limit Switch
  limitSwitch.loop();

  //Encoder Switch
  int btnState = digitalRead(SW);
  if (btnState == LOW) {
		if (millis() - lastButtonPress > 300) {
      Menu = Menu + 1;
			//Serial.println(Menu);      
		lastButtonPress = millis();
	  }
    }
  //fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
  if ( Menu == 0 ){
            display.clearDisplay();
            //Serial.println("=====MAIN SCREEN=====^");
            display.setCursor(0,0);
            display.println("=====MAIN SCREEN=====");
            display.setCursor(0,10);
            display.print("1). Distance = ");
            display.println(X1); //X1 is distance value variable
            display.setCursor(0,19);
            display.print("2). No. Cycle = ");
            display.println(NoC);
            display.setCursor(0,28);
            display.print("3). Speed = ");
              if(Fast == true){
                //Serial.print("Fast");
                display.println("Fast");
              } else {
                //Serial.println("Slow");
                display.println("Slow");
              }
            display.setCursor(0,35);
            display.print("4). Move = ");
              if(MOVE == true){
                //Serial.print("YES");
                display.println("YES");
              } else {
                //Serial.println("NO");
                display.println("NO");
              }
            display.setCursor(0, 56);
            display.print("STATUS = ");
            display.fillRect(60, 56, 80, 8, SH110X_BLACK); // create black rect at cursor to overwrite the text
            display.setCursor(60,56);
            display.println("Ready..");
            Wire.begin();
            Wire.beginTransmission(MPU_addr);
            Wire.write(0x3B);
            Wire.endTransmission(false);
            Wire.requestFrom(MPU_addr,6,true);
            GyX=Wire.read()<<8|Wire.read();
            GyY=Wire.read()<<8|Wire.read();
            GyZ=Wire.read()<<8|Wire.read();
            int xAng = map(GyX,minVal,maxVal,-90,90);
            int yAng = map(GyY,minVal,maxVal,-90,90);
            int zAng = map(GyZ,minVal,maxVal,-90,90);
            z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
            int Z1 = z;
            int Z2 = (z-Z1); // getting decimal data
            Z3 = z-Z2; // subtracting decimal data form raw data
            display.setCursor(0, 44);
            display.print("5). Angle = ");
              if(Z3 >= 180){
                Z3 = Z3 - 360;
                display.println(Z3);
              } else {
              display.println(Z3);
              }
            display.display();
        if(Fast == true){
            Speed = 100;
          } else {
            Speed = 1000;
        }
          if(MOVE == true){
            digitalWrite(dirPin, HIGH);
              for(int x = 0; x < Distance; x++)
                {
                digitalWrite(stepPin, HIGH);
                delay(Speed);
                digitalWrite(stepPin, LOW); 
                delay(Speed);
                display.fillRect(60, 56, 80, 8, SH110X_BLACK);
                display.setCursor(60,56);
                display.print("Moving_");
                display.println(Distance);
                display.display();
                }
                MOVE = false;
                display.fillRect(60, 56, 80, 8, SH110X_BLACK);
                display.setCursor(60,56);
                display.print("Moved");
                display.display();
                delay(2000);
          }
        else{
          //Does nothing for now
          digitalWrite(stepPin, LOW);
        }
          if(START == true){
            //going home
                display.fillRect(60, 56, 80, 8, SH110X_BLACK);
                display.setCursor(60,56);
                display.print("Running");
                display.display();
                int state = limitSwitch.getState();
            for(CC = 1;CC<=NoC;CC++){
              do {
                    digitalWrite(dirPin, HIGH);
                    digitalWrite(stepPin, HIGH);
                    delay(Speed);
                    digitalWrite(stepPin, LOW); 
                    delay(Speed);
                    Wire.begin();
                    Wire.beginTransmission(MPU_addr);
                    Wire.write(0x3B);
                    Wire.endTransmission(false);
                    Wire.requestFrom(MPU_addr,6,true);
                    GyX=Wire.read()<<8|Wire.read();
                    GyY=Wire.read()<<8|Wire.read();
                    GyZ=Wire.read()<<8|Wire.read();
                    int xAng = map(GyX,minVal,maxVal,-90,90);
                    int yAng = map(GyY,minVal,maxVal,-90,90);
                    int zAng = map(GyZ,minVal,maxVal,-90,90);
                    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
                    int Z1 = z;
                    int Z2 = (z-Z1); // getting decimal data
                    Z3 = z-Z2; // subtracting decimal data form raw data
                    display.fillRect(64, 44, 80, 8, SH110X_BLACK);
                    display.setCursor(0, 44);
                    display.print("5). Angle = ");
                      if(Z3 >= 180){
                        Z3 = Z3 - 360;
                        display.println(Z3);
                      } else {
                      display.println(Z3);
                      }
                    display.display();
                    state = digitalRead(7);
                  } 
                  while(state == HIGH);
                  if(state == LOW){
                    digitalWrite(dirPin, LOW);
                    for(int x = 0; x < Distance; x++)
                      {
                        digitalWrite(stepPin, HIGH);
                        delay(Speed);
                        digitalWrite(stepPin, LOW); 
                        delay(Speed);
                        Wire.begin();
                        Wire.beginTransmission(MPU_addr);
                        Wire.write(0x3B);
                        Wire.endTransmission(false);
                        Wire.requestFrom(MPU_addr,6,true);
                        GyX=Wire.read()<<8|Wire.read();
                        GyY=Wire.read()<<8|Wire.read();
                        GyZ=Wire.read()<<8|Wire.read();
                        int xAng = map(GyX,minVal,maxVal,-90,90);
                        int yAng = map(GyY,minVal,maxVal,-90,90);
                        int zAng = map(GyZ,minVal,maxVal,-90,90);
                        z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
                        int Z1 = z;
                        int Z2 = (z-Z1); // getting decimal data
                        Z3 = z-Z2; // subtracting decimal data form raw data
                        display.fillRect(64, 44, 80, 8, SH110X_BLACK);
                        display.setCursor(0, 44);
                        display.print("5). Angle = ");
                          if(Z3 >= 180){
                            Z3 = Z3 - 360;
                            display.println(Z3);
                          } else {
                          display.println(Z3);
                          }
                        display.display();
                      }
                      state = HIGH;
                      display.fillRect(0, 19, 110, 8, SH110X_BLACK);
                      display.setCursor(0,19);
                      display.print("2). Cycle = ");
                      display.println(CC);
                      display.display();
                  } 
            }
                START = false; //stops the experiment when cycles are completed
          }
   }
  if ( Menu == 1 ){ 
            display.clearDisplay();
            //Serial.println("=====MAIN SCREEN======");
            display.setCursor(0,0);
            display.println("=====MAIN SCREEN2====");
            display.setCursor(4,13);
            display.print("  Distance = ");
            display.println(X1);
            display.setCursor(4,24);
            display.print("  Cycles = ");
            display.println(X2);
            display.setCursor(4,45);
            display.print("  Speed = ");
              if(Fast == true){
                //Serial.print("Fast");
                display.println("Fast");
              } else {
                //Serial.println("Slow");
                display.println("Slow");
              }
            display.setCursor(4,34);
            display.print("  Move = ");
              if(MOVE == true){
                //Serial.print("YES");
                display.println("YES");
              } else {
                //Serial.println("NO");
                display.println("NO");
              }
            GyX=Wire.read()<<8|Wire.read();
            GyY=Wire.read()<<8|Wire.read();
            GyZ=Wire.read()<<8|Wire.read();
            int xAng = map(GyX,minVal,maxVal,-90,90);
            int yAng = map(GyY,minVal,maxVal,-90,90);
            int zAng = map(GyZ,minVal,maxVal,-90,90);
            z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
            int Z1 = z;
            int Z2 = (z-Z1); // getting decimal data
            int Z3 = z-Z2; // subtracting decimal data form raw data
            display.setCursor(0,56);
            display.print("   Angle = ");
            if(Z3 >= 180){
              Z3 = Z3 - 360;
              display.println(Z3);
            } else {
            display.println(Z3);
            }
            display.display();
      }
    
  if ( Menu == 2 ){ 
        Distance = 200*X1; //in mm per rotation
        display.clearDisplay();
        //Serial.print("> Distance = ");
        //Serial.println(X1);
        display.setCursor(0,0);
        display.println("=====SET VALUES======");
        display.setCursor(4,13);
        display.print("> Distance = ");
        display.println(X1);
        display.setCursor(4,24);
        display.print("  Cycles = ");
        display.println(X2);
        display.setCursor(4,34);
        display.print("  Move = ");
          if(MOVE == true){
            //Serial.print("YES");
            display.println("YES");
          } else {
            //Serial.println("NO");
            display.println("NO");
          }
        display.setCursor(4,45);
        display.print("  Speed = ");
          if(Fast == true){
            //Serial.print("Fast");
            display.println("Fast");
          } else {
            //Serial.println("Slow");
            display.println("Slow");
          }
        display.setCursor(4,56);
        display.print("  START = ");
          if(START == true){
            //Serial.print("YES");
            display.println("YES");
          } else {
            //Serial.println("NO");
            display.println("NO");
          }
        display.display();
      }

  if ( Menu == 3 ){ 
        NoC = X2;
        display.clearDisplay();
        //Serial.print("  Cycles = ");
        //Serial.println(X2);
        display.setCursor(0,0);
        display.println("=====SET VALUES======");
        display.setCursor(4,13);
        display.print("  Distance = ");
        display.println(X1);
        display.setCursor(4,24);
        display.print("> Cycles = ");
        display.println(X2);
        display.setCursor(4,45);
        display.print("  Speed = ");
          if(Fast == true){
            //Serial.print("Fast");
            display.println("Fast");
          } else {
            //Serial.println("Slow");
            display.println("Slow");
          }
        display.setCursor(4,34);
        display.print("  Move = ");
          if(MOVE == true){
            //Serial.print("YES");
            display.println("YES");
          } else {
           // Serial.println("NO");
            display.println("NO");
          }
        display.setCursor(4,56);
        display.print("  START = ");
          if(START == true){
           // Serial.print("YES");
            display.println("YES");
          } else {
          //  Serial.println("NO");
            display.println("NO");
          }
        display.display();
    }

  if ( Menu == 4 ){ 
        display.clearDisplay();
       // Serial.print("  MOVE");
        display.setCursor(0,0);
        display.println("=====SET VALUES======");
        display.setCursor(4,13);
        display.print("  Distance = ");
        display.println(X1);
        display.setCursor(4,24);
        display.print("  Cycles = ");
        display.println(X2);
        display.setCursor(4,45);
        display.print("  Speed = ");
          if(Fast == true){
          //  Serial.print("Fast");
            display.println("Fast");
          } else {
          //  Serial.println("Slow");
            display.println("Slow");
          }
        display.setCursor(4,34);
        display.print("> Move = ");
          if(MOVE == true){
          //  Serial.print("YES");
            display.println("YES");
          } else {
          //  Serial.println("NO");
            display.println("NO");
          }
        display.setCursor(4,56);
        display.print("  START = ");
          if(START == true){
          //  Serial.print("YES");
            display.println("YES");
          } else {
          //  Serial.println("NO");
            display.println("NO");
          }
        display.display();
    }

  if ( Menu == 5 ){ 
        display.clearDisplay();
      //  Serial.print("  Speed");
        display.setCursor(0,0);
        display.println("=====SET VALUES======");
        display.setCursor(4,13);
        display.print("  Distance = ");
        display.println(X1);
        display.setCursor(4,24);
        display.print("  Cycles = ");
        display.println(X2);
        display.setCursor(4,45);
        display.print("> Speed = ");
          if(Fast == true){
          //  Serial.print("Fast");
            display.println("Fast");
          } else {
          //  Serial.println("Slow");
            display.println("Slow");
          }
        display.setCursor(4,34);
        display.print("  Move = ");
          if(MOVE == true){
          //  Serial.print("YES");
            display.println("YES");
          } else {
          //  Serial.println("NO");
            display.println("NO");
          }
        display.setCursor(4,56);
        display.print("  START = ");
          if(START == true){
          //  Serial.print("YES");
            display.println("YES");
          } else {
          //  Serial.println("NO");
            display.println("NO");
          }
        display.display();
    }
  if ( Menu == 6 ){ 
        display.clearDisplay();
      //  Serial.print("  START");
        display.setCursor(0,0);
        display.println("=====RUN PROGRAM=====");
        display.setCursor(4,13);
        display.print("  Distance = ");
        display.println(X1);
        display.setCursor(4,24);
        display.print("  Cycles = ");
        display.println(X2);
        display.setCursor(4,45);
        display.print("  Speed = ");
          if(Fast == true){
          //  Serial.print("Fast");
            display.println("Fast");
          } else {
          //  Serial.println("Slow");
            display.println("Slow");
          }
        display.setCursor(4,34);
        display.print("  Move = ");
          if(MOVE == true){
          //  Serial.print("YES");
            display.println("YES");
          } else {
          //  Serial.println("NO");
            display.println("NO");
          }
        display.setCursor(4,56);
        display.print("> START = ");
          if(START == true){
          //  Serial.print("YES");
            display.println("YES");
          } else {
          //  Serial.println("NO");
            display.println("NO");
          }
        display.display();
    }

  if ( Menu >= 7 ){
    if(START == true){
      Menu = 0;
    } else{
      Menu = 1;
    }
   }
        delay(1);
   }
//ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
void handleEncoder(){
  if ( Menu == 2 ){ //Input for distance for stretching
    currentStateCLK = digitalRead(CLK);
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      if (digitalRead(DT) != currentStateCLK) {
        X1 --;
      } else {
        X1 ++;
      }
    }
    lastStateCLK = currentStateCLK;
  }

  if ( Menu == 3 ){ //Input for number of cycles
    currentStateCLK = digitalRead(CLK);
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      if (digitalRead(DT) != currentStateCLK) {
        X2 --;
      } else {
        X2 ++;
      }
    }
    lastStateCLK = currentStateCLK;
  }

  if ( Menu == 4 ){ //To move back the clamp
    currentStateCLK = digitalRead(CLK);
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      if (digitalRead(DT) != currentStateCLK) {
        MOVE = false;
      } else {
        MOVE = true;
      }
    }
    lastStateCLK = currentStateCLK;
  }

  if ( Menu == 5 ){ 
    currentStateCLK = digitalRead(CLK);
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      if (digitalRead(DT) != currentStateCLK) {
        Fast = false;
      } else {
        Fast = true;
      }
    }
    lastStateCLK = currentStateCLK;
  }

  if ( Menu == 6 ){ // to start the program
    currentStateCLK = digitalRead(CLK);
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      if (digitalRead(DT) != currentStateCLK) {
        START = false;
      } else {
        START = true;
      }
    }
    lastStateCLK = currentStateCLK;
  }
 }