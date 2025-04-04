#include <IRremote.hpp>
#include <LiquidCrystal_I2C.h>
#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
#define IR_RECEIVE_PIN 7

#define AUT_PIN 4
#define P1_PIN 8
#define M1_PIN 2

int gainValue = 0;
int heelValue = 0;

int buttonAutValue = 1;
int buttonPlusOneValue = 1;
int buttonMinusOneValue = 1;

int gainPotardValue = 0;
int heelPotardValue = 0;

int status = 0; // Stdb
int HDG = 0;
int cap = 0; 
int heel = 0;

LiquidCrystal_I2C lcd(0x27,16,4);  
MPU9250 IMU; 

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
MagData magData;

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(5000);
  }

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");

  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver

  pinMode(AUT_PIN, INPUT_PULLUP);
  pinMode(P1_PIN, INPUT_PULLUP);
  pinMode(M1_PIN, INPUT_PULLUP);

  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
}

void loop() {

  updateSettings();
  keysHandler();
  updateDisplay();
  applyCorrection();
}

void applyCorrection()
{
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getMag(&magData);

  float magX = magData.magX;
  float magY = magData.magY;
  float magZ = magData.magZ;

  float accX = accelData.accelX;
  float accY = accelData.accelY;
  float accZ = accelData.accelZ;

  float roll = atan2(accY, accZ);
  float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ));

  float magX_corrected = magX * cos(pitch) + magZ * sin(pitch);
  float magY_corrected = magX * sin(roll) * sin(pitch) + magY * cos(roll) - magZ * sin(roll) * cos(pitch);

  float heading_angle_in_degrees = atan2(magY_corrected, magX_corrected) * (180 / 3.141592);


  if (heading_angle_in_degrees < 0) {
      heading_angle_in_degrees += 360.0;
  }

  //Serial.println(heading_angle_in_degrees);
  HDG = (int)heading_angle_in_degrees;
  heel = (int)accelData.accelX;
  if (IMU.hasTemperature()) {
	  //Serial.print("\t");
	 // Serial.println(IMU.getTemp());
  }
  else {
    Serial.println();
  }
  delay(50);
}


void keysHandler()
{

    if(digitalRead(AUT_PIN) != buttonAutValue && digitalRead(AUT_PIN) == 0)
    {
      Serial.println("HARD AUTO");

        lcd.setCursor(2,3);   
        lcd.print(F("Received: AUT"));
        status = 1;

      buttonAutValue = 0;
    }
    else if(digitalRead(AUT_PIN) != buttonAutValue)
    {
      buttonAutValue = 1;
    }
    
    if(digitalRead(P1_PIN) != buttonPlusOneValue && digitalRead(P1_PIN) == 0)
    {
      Serial.println("HARD +1");

        lcd.setCursor(2,3);   
        cap = cap + 1;

      buttonPlusOneValue = 0;
    }
    else if(digitalRead(P1_PIN) != buttonPlusOneValue)
    {
      buttonPlusOneValue = 1;
    }

    if(digitalRead(M1_PIN) != buttonMinusOneValue && digitalRead(M1_PIN) == 0)
    {
      Serial.println("HARD -1");

        lcd.setCursor(2,3);   
        cap = cap - 1;

      buttonMinusOneValue = 0;
    }
    else if(digitalRead(M1_PIN) != buttonMinusOneValue)
    {
      buttonMinusOneValue = 1;
    }



    if (IrReceiver.decode()) {

      if(IrReceiver.decodedIRData.decodedRawData == 0xA55AFF00)
      {
        Serial.println("+1");
        cap = cap + 1;
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xE916FF00)
      {
        Serial.println("AUT");
        status = 1;
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xF708FF00)
      {
        Serial.println("-1");
        cap = cap - 1;
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xE718FF00)
      {
        Serial.println("+10");
        cap = cap + 10;
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xF20DFF00)
      {
        Serial.println("SDB");
        status = 0; 
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xAD52FF00)
      {
        Serial.println("-10");
        cap = cap - 10;
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xBA45FF00)
      {
        Serial.println("+1 Gain");
        if(gainValue < 10)
        {
          gainValue = gainValue +1;
        }
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xBB44FF00)
      {
        Serial.println("-1 Gain");
        if(gainValue > 0)
        {
          gainValue = gainValue -1;
        }
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xB847FF00)
      {
        Serial.println("+1 Gite");
        if(heelValue < 10)
        {
          heelValue = heelValue +1;
        }
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xBC43FF00)
      {
        Serial.println("-1 Gite");
        if(heelValue > 0)
        {
          heelValue = heelValue -1;
        }
      }


      IrReceiver.resume(); // Enable receiving of the next value
  }

      if(cap == 360) 
      {
        cap = 0;
      }
      if(cap > 360) 
      {
        cap = cap - 360;
      }
      if(cap < 0) 
      {
        cap = 360 + cap;
      }
}


void updateSettings()
{
  if(gainPotardValue > analogRead(A0)+20 || gainPotardValue < analogRead(A0)-20 )
  {
    gainValue = analogRead(A0)/110;
    gainPotardValue = analogRead(A0);
  }
  if(heelPotardValue > analogRead(A1)+20 || heelPotardValue < analogRead(A1)-20 )
  {
     heelValue = analogRead(A1)/110;
     heelPotardValue = analogRead(A1);
  }
}


int Delta()
{
  int delta = 666;
    lcd.setCursor(8,1);
    lcd.print(F("D"));
    if(delta < 10)
    {  
      lcd.print(F("00"));
      lcd.print(delta);
    }
    else if(delta < 100)
    {  
      lcd.print(F("0"));
      lcd.print(delta);
    }
    else
    {  
      lcd.print(delta);
    }

    return delta;
}
void updateDisplay()
{
  if(status == 0)
  {
    lcd.setCursor(0,0);   
    lcd.print(F("SDB"));
  }
  else if(status == 1)
  {
    lcd.setCursor(0,0);   
    lcd.print(F("AUT"));
  }


    lcd.setCursor(4,0);   
    if(cap < 10)
    {  
      lcd.print(F("00"));
      lcd.print(cap);
    }
    else if(cap < 100)
    {  
      lcd.print(F("0"));
      lcd.print(cap);
    }
    else
    {  
      lcd.print(cap);
    }


    lcd.setCursor(8,0);
    lcd.print(F("G:"));
    lcd.setCursor(10,0);
    lcd.print(gainValue);

    lcd.setCursor(13,0);
    lcd.print(F("H:"));
    lcd.setCursor(15,0);
    lcd.print(heelValue);


    lcd.setCursor(0,1);
    lcd.print(F("HDG:"));
    if(HDG < 10)
    {  
      lcd.print(F("00"));
      lcd.print(HDG);
    }
    else if(HDG < 100)
    {  
      lcd.print(F("0"));
      lcd.print(HDG);
    }
    else
    {  
      lcd.print(HDG);
    }

    lcd.setCursor(13,1);
    lcd.print(F("Hl:"));
    lcd.setCursor(17,1);
    lcd.print(heel);
}
