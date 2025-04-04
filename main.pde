#include <Wire.h>
#include <LiquidCrystal_I2C.h>                                          // YwRobot Arduino LCM1602 IIC V1 library
#include <IRremote.hpp>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// ----- LCD
LiquidCrystal_I2C lcd(0x27, 16, 4);          // LCD pinouts: addr,en,rw,rs,d4,d5,d6,d7,bl,blpol
int LCD_loop_counter;
int LCD_heading_buffer, LCD_pitch_buffer, LCD_roll_buffer;

// ----- Gyro
#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250 
#define MPU9250_I2C_master_enable 0x6A                                  // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN


#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

#define Frequency 125                                                   // 8mS sample interval 
#define Sensitivity 65.5                                                // Gyro sensitivity (see data sheet)

#define Sensor_to_deg 1/(Sensitivity*Frequency)                         // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg*DEG_TO_RAD                          // Convert sensor reading to radians

#define Loop_time 1000000/Frequency                                     // Loop time (uS)
long    Loop_start;                                                     // Loop start time (uS)

int     Gyro_x,     Gyro_y,     Gyro_z;
long    Gyro_x_cal, Gyro_y_cal, Gyro_z_cal;
float   Gyro_pitch, Gyro_roll, Gyro_yaw;
float   Gyro_pitch_output, Gyro_roll_output;

// ----- Accelerometer
long    Accel_x,      Accel_y,      Accel_z,    Accel_total_vector;
float   Accel_pitch,  Accel_roll;

// ----- Magnetometer
#define AK8963_I2C_address 0x0C                                             // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A                                             // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                            // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                   // Data ready mask
#define AK8963_overflow_mask 0b00001000                                     // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                    // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10                                                // X,Y,Z fuse ROM


float   Declination = -13;                                             //  Degrees ... replace this declination with yours
int     Heading;

int     Mag_x,                Mag_y,                Mag_z;                  // Raw magnetometer readings
float   Mag_x_dampened,       Mag_y_dampened,       Mag_z_dampened;
float   Mag_x_hor, Mag_y_hor;
float   Mag_pitch, Mag_roll;

// ----- Record compass offsets, scale factors, & ASA values
/*
   These values seldom change ... an occasional check is sufficient
   (1) Open your Arduino "Serial Monitor
   (2) Set "Record_data=true;" then upload & run program.
   (3) Replace the values below with the values that appear on the Serial Monitor.
   (4) Set "Record_data = false;" then upload & rerun program.
*/
bool    Record_data = false;
int     Mag_x_offset = 172,      Mag_y_offset = 256,     Mag_z_offset = -270;   // Hard-iron offsets
float   Mag_x_scale = 0.96,     Mag_y_scale = 0.92,     Mag_z_scale = 1.04;    // Soft-iron scale factors
float   ASAX = 1.19,            ASAY = 1.20,            ASAZ = 1.16;           // (A)sahi (S)ensitivity (A)djustment fuse ROM values.

// ----- LED
const int LED = 13;                     // Status LED

// ----- Flags
bool Gyro_synchronised = false;
bool Flag = false;

// ----- Debug

long Loop_start_time;
long Debug_start_time;

long secondCounter;
long secondCounterTillHeaded;

#define IR_RECEIVE_PIN 7

#define AUT_PIN 4
#define MODE_PIN 9
#define P1_PIN 8
#define M1_PIN 2

int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)


static const int RXPin = 12, TXPin = 11;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


int sats = 0;
float lat = 0;
float lng = 0;
int cog = 0;
float sog = 0;

float angleBarre = 0.0;
int oldHeading = 0;
int oldDelta = 0;
int oldMag_roll = 0;
int oldAngleBarre = 0;

int gainValue = 0;
float heelValue = 0;

int buttonAutValue = 1;
int buttonModValue = 1;
int buttonPlusOneValue = 1;
int buttonMinusOneValue = 1;

int gainPotardValue = 0;
int heelPotardValue = 0;

int status = 0; // Stdb
int mode = 0; //cog mag
int cap = 0; 
int heel = 0;
int delta = 0;
float errorDelta = 0;
bool firstFixReceived = false;
int integralDelta = 0;
int lastDelta = 0;


unsigned long previousMillis = 0;
const long interval = 1000;

void setup()
{
  Serial.begin(115200);                                 
  Wire.begin();                                         
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  Wire.setClock(400000);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  lcd.init();
  lcd.clear();         
  lcd.backlight();


  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);


  configure_magnetometer();
  if (Record_data == true)
  {
    calibrate_magnetometer();
  }
  config_gyro();
  calibrate_gyro();

  lcd.clear();  
  if (status == 0)
  {
    lcd.setCursor(0,0);   
    lcd.print(F("SDB"));
  }
  else
  {
    lcd.setCursor(0,0);   
    lcd.print(F("AUT"));
  }

  if (mode == 0)
  {
    lcd.setCursor(4,0);   
    lcd.print(F("MAG"));
  }
  else
  {
    lcd.setCursor(4,0);   
    lcd.print(F("GPS"));
  }
  lcd.setCursor(8,0);   
  lcd.print(F("Gain:"));
  lcd.setCursor(15,0);   
  lcd.print(F("O:"));

  lcd.setCursor(0,1);   
  lcd.print(F("H:"));
  lcd.setCursor(6,1);   
  lcd.print(F("C:"));
  lcd.setCursor(12,1);   
  lcd.print(F("SP:"));

  lcd.setCursor(0,2);   
  lcd.print(F("DT:"));
  lcd.setCursor(8,2);   
  lcd.print(F("HL:"));
  lcd.setCursor(16,2);   
  lcd.print(F("S:"));

  lcd.setCursor(0,3); 
  lcd.print(F("PID:"));

  lcd.setCursor(13,0); 
  lcd.print(gainValue);

  lcd.setCursor(17,0); 
  lcd.print(heelValue);


  lcd.setCursor(2,1);   
  lcd.print(F("---"));
  lcd.setCursor(8,1);   
  lcd.print(F("---"));
  lcd.setCursor(15,1);   
  lcd.print(F("-----"));

  lcd.setCursor(18,2);   
  lcd.print(F("--"));


  ss.begin(GPSBaud);

  Debug_start_time = micros();                          // Controls data display rate to Serial Monitor
  Loop_start_time = micros();                           // Controls the Gyro refresh rate
  secondCounter = 0;
  secondCounterTillHeaded = 0;
}


void loop()
{
  updatePotentiometers();
  keysHandler();


  if(mode == 0)
  {
    computeCompass();
    while ((micros() - Loop_start_time) < 8000);
    Loop_start_time = micros();
  }
  else if(mode == 1)
  {
    smartDelay(1000);
    fetchGPSSensor();
  }


  unsigned long currentMillis = millis();

  

  if (currentMillis - previousMillis >= 500) 
  {
    previousMillis = currentMillis;
    processDelta();
  }

  secondCounter =  millis() / 1000;


}

void processDelta()
{
  int currentHeading = 0;
  if(mode == 0)
  {
    currentHeading = Heading;
  }
  else
  {
    currentHeading = cog;
  }

  delta = cap - currentHeading;
    
  if (delta > 180) {
      delta -= 360;
  } else if (delta < -180) {
      delta += 360;
  }

  

  if (delta < 3 && delta > -3) 
  {
    secondCounterTillHeaded = secondCounter;
    integralDelta = 0;
  }

  if(delta != oldDelta)
  {
    int deltaAbs = abs(delta);

    if(delta > 0)
    {
      lcd.setCursor(3,2); 
      lcd.print(F("+"));
    }
    else
    {
      lcd.setCursor(3,2); 
      lcd.print(F("-"));
    }

    if(delta < 10)
    {
      lcd.setCursor(4,2); 
      lcd.print(deltaAbs);
      lcd.print(F("  "));
    }
    else if(delta < 100)
    {
      lcd.setCursor(4,2); 
      lcd.print(deltaAbs);
      lcd.print(F(" "));
    }
    else 
    {
      lcd.setCursor(4,2); 
      lcd.print(deltaAbs);
    }

    oldDelta = delta;
  }


  float Kp = 0.7;
  float Ki = 0.1;
  float Kd = 0.3;

  // Nombre de secondes depuis qu'on est offroute t
  // Quand on appuie sur AUT, ça lance le timer qui ne se reset que quand delta <= 3


  
  if(integralDelta > -800 && integralDelta < 800) 
  {
    integralDelta += delta;
  }

  float deltaT;

  if(mode == 0) 
  {
    deltaT = 0.5;
  }
  else if(mode == 1) 
  {
    deltaT = 1;
  }


  
  float P = delta * Kp ;
  float I = ((delta + integralDelta) * deltaT) * Ki ;
  float D = ((abs(delta) - abs(lastDelta)) / deltaT) * Kd ;

  if(delta < 0)
    D = -D;

  float PID = P + I + D;
  float K = 0.1 + (heelValue/10);
  PID = PID * K;

  PID = -PID;



  


  String sortie;
  sortie.reserve(21);
  sortie += "P";
  sortie += (String)(int)P;
  sortie += " I";
  sortie += (String)(int)I;
  sortie += " D";
  sortie += (String)(int)D;
  sortie += " T";
  sortie += (String)(int)PID;

  
  Serial.println(sortie);


  lcd.setCursor(0,3); 
  lcd.println(sortie);



  if(PID < -20)
    PID = -20;

  if(PID > 20)
    PID = 20;

  int vitesseMoteur = map(gainValue, 0, 9, 0, 120);

  if(PID < 1 && PID > -1) 
  {
    //Stop PWM
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);
  }
  else if(PID > 2 && PID < 12)
  {
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, vitesseMoteur);
  }
  else if(PID < -2 && PID > -12)
  {
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, vitesseMoteur);
  }
  else if(PID >= 12)
  {
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, vitesseMoteur*2);
  }
  else if(PID <= -12)
  {
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, vitesseMoteur*2);
  }

  lastDelta = delta;
}


void fetchGPSSensor()
{
    // Sattelites
  if(gps.satellites.isValid()) 
  {
    sats = gps.satellites.value();
  }

  // Coords
  if(gps.location.isValid()) 
  {
    lat = gps.location.lat();
    lng = gps.location.lng();
  }
    // Cog
  if(gps.course.isValid()) 
  {
    cog = gps.course.deg();
  }

    // Sog
  if(gps.speed.isValid()) 
  {
    sog = gps.speed.knots();
  }

  lcd.setCursor(18,2);   
  
  if(sats > 9)
  {
    lcd.print(sats);
  }
  else
  {
    lcd.print(sats);
    lcd.setCursor(19,2);
    lcd.print(" ");
  }

  lcd.setCursor(15,1);   
  lcd.print(sog);

  if(cog < 10)
  {
    lcd.setCursor(8,1);
    lcd.print(cog);
    lcd.print(F("  "));
  }
  else if(cog < 100)
  {
    lcd.setCursor(8,1);
    lcd.print(cog);
    lcd.print(F(" "));
  }
  else 
  {
    lcd.setCursor(8,1);
    lcd.print(cog);
  }

  
  
}




void keysHandler()
{
/*
    if(digitalRead(AUT_PIN) != buttonAutValue && digitalRead(AUT_PIN) == 0)
    {
      if(status == 1) 
      {
        status = 0;
        Serial.println("HARD SDB");
      }
      else if(status == 0) 
      {
        status = 1;
        cap = Heading;
        Serial.println("HARD AUTO");
      }
      buttonAutValue = 0;
    }
    else if(digitalRead(AUT_PIN) != buttonAutValue)
    {
      buttonAutValue = 1;
    }
    
    if(digitalRead(MODE_PIN) != buttonModValue && digitalRead(MODE_PIN) == 0)
    {
       if(mode == 1) 
      {
        mode = 0;
        Serial.println("HARD MAG");
      }
      else if(mode == 0) 
      {
        mode = 1;
        Serial.println("HARD COG");
      }
      buttonModValue = 0;
    }
    else if(digitalRead(MODE_PIN) != buttonModValue)
    {
      buttonModValue = 1;
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
        cap = cap - 1;

      buttonMinusOneValue = 0;
    }
    else if(digitalRead(M1_PIN) != buttonMinusOneValue)
    {
      buttonMinusOneValue = 1;
    }

    */

    if (IrReceiver.decode()) {
      // Serial.println(IrReceiver.decodedIRData.decodedRawData);
      if(IrReceiver.decodedIRData.decodedRawData == 0xA55AFF00)
      {
        Serial.println("+1");
        cap = cap + 1;
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xE916FF00)
      {
        Serial.println("AUT");
        status = 1;
        cap = Heading;
        lcd.setCursor(0,0);   
        lcd.print(F("AUT"));
        secondCounterTillHeaded = secondCounter;
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
        lcd.setCursor(0,0);   
        lcd.print(F("SDB"));
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
        Serial.println("+1 Barre");
        if(heelValue < 10)
        {
          heelValue = heelValue +1;
        }
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 0xBC43FF00)
      {
        Serial.println("-1 Barre");
        if(heelValue > 0)
        {
          heelValue = heelValue -1;
        }
      }
      else if(IrReceiver.decodedIRData.decodedRawData == 3810328320)
      {
        if(mode == 0)
        {
          mode = 1;
          Serial.println("Mode GPS");
          lcd.setCursor(4,0);   
          lcd.print(F("GPS"));

          lcd.setCursor(2,1);   
          lcd.print(F("---"));
        }
        else
        {
          mode = 0;
          Serial.println("Mode MAG");
          lcd.setCursor(4,0);   
          lcd.print(F("MAG"));
          lcd.setCursor(8,1);   
          lcd.print(F("---"));
          lcd.setCursor(15,1);   
          lcd.print(F("-----"));
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


void updatePotentiometers()
{
  if(gainPotardValue > analogRead(A0)+20 || gainPotardValue < analogRead(A0)-20 )
  {
    gainValue = analogRead(A0)/110;
    gainPotardValue = analogRead(A0);
    lcd.setCursor(13,0);   
    lcd.print(gainValue);
  }
  if(heelPotardValue > analogRead(A1)+20 || heelPotardValue < analogRead(A1)-20 )
  {
     heelValue = analogRead(A1)/110;
     heelPotardValue = analogRead(A1);
     lcd.setCursor(17,0); 
     lcd.print(heelValue);
     
  }
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}







void computeCompass()
{
    read_mpu_6050_data();                                             // Read the raw acc and gyro data from the MPU-6050


  Gyro_x -= Gyro_x_cal;                                             // Subtract the offset from the raw gyro_x value
  Gyro_y -= Gyro_y_cal;                                             // Subtract the offset from the raw gyro_y value
  Gyro_z -= Gyro_z_cal;                                             // Subtract the offset from the raw gyro_z value

  Gyro_pitch += -Gyro_y * Sensor_to_deg;                            // Integrate the raw Gyro_y readings
  Gyro_roll += Gyro_x * Sensor_to_deg;                              // Integrate the raw Gyro_x readings
  Gyro_yaw += -Gyro_z * Sensor_to_deg;                              // Integrate the raw Gyro_x readings

  // ----- Compensate pitch and roll for gyro yaw
  Gyro_pitch += Gyro_roll * sin(Gyro_z * Sensor_to_rad);            // Transfer the roll angle to the pitch angle if the Z-axis has yawed
  Gyro_roll -= Gyro_pitch * sin(Gyro_z * Sensor_to_rad);            // Transfer the pitch angle to the roll angle if the Z-axis has yawed

  // ----- Accelerometer angle calculations
  Accel_total_vector = sqrt((Accel_x * Accel_x) + (Accel_y * Accel_y) + (Accel_z * Accel_z));   // Calculate the total (3D) vector
  Accel_pitch = asin((float)Accel_x / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the pitch angle
  Accel_roll = asin((float)Accel_y / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the roll angle


  Accel_pitch -= -0.2f;                                             //Accelerometer calibration value for pitch
  Accel_roll -= 1.1f;                                               //Accelerometer calibration value for roll

  // ----- Correct for any gyro drift
  if (Gyro_synchronised)
  {
    // ----- Gyro & accel have been synchronised
    Gyro_pitch = Gyro_pitch * 0.9996 + Accel_pitch * 0.0004;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    Gyro_roll = Gyro_roll * 0.9996 + Accel_roll * 0.0004;           //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {
    // ----- Synchronise gyro & accel
    Gyro_pitch = Accel_pitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll = Accel_roll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_synchronised = true;                                             //Set the IMU started flag
  }

  // ----- Dampen the pitch and roll angles
  Gyro_pitch_output = Gyro_pitch_output * 0.9 + Gyro_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  Gyro_roll_output = Gyro_roll_output * 0.9 + Gyro_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value



  read_magnetometer();


  Mag_pitch = -Gyro_roll_output * DEG_TO_RAD;
  Mag_roll = Gyro_pitch_output * DEG_TO_RAD;

  Mag_x_hor = Mag_x * cos(Mag_pitch) + Mag_y * sin(Mag_roll) * sin(Mag_pitch) - Mag_z * cos(Mag_roll) * sin(Mag_pitch);
  Mag_y_hor = Mag_y * cos(Mag_roll) + Mag_z * sin(Mag_roll);

  Mag_x_dampened = Mag_x_dampened * 0.9 + Mag_x_hor * 0.1;
  Mag_y_dampened = Mag_y_dampened * 0.9 + Mag_y_hor * 0.1;

  Heading = atan2(Mag_x_dampened, Mag_y_dampened) * RAD_TO_DEG;  // Magnetic North

  if (Heading > 360.0) Heading -= 360.0;
  if (Heading < 0.0) Heading += 360.0;

  if (Heading < 0) Heading += 360;
  if (Heading >= 360) Heading -= 360;

  
  if(Heading != oldHeading)
  {
    if(Heading < 10)
    {
      lcd.setCursor(2,1);
      lcd.print(Heading);
      lcd.print(F("  "));
    }
    else if(Heading < 100)
    {
      lcd.setCursor(2,1);
      lcd.print(Heading);
      lcd.print(F(" "));
    }
    else 
    {
      lcd.setCursor(2,1);
      lcd.print(Heading);
    }

    oldHeading = Heading;
  }

  


  if(Accel_roll != oldMag_roll)
  {
    int Accel_rollAbs = abs(Accel_roll);

    if(Accel_roll > 0)
    {
      lcd.setCursor(11,2); 
      lcd.print(F("+"));
    }
    else
    {
      lcd.setCursor(11,2); 
      lcd.print(F("-"));
    }

    if(Accel_roll < 10)
    {
      lcd.setCursor(12,2); 
      lcd.print(Accel_rollAbs);
      lcd.print(F("  "));
    }
    else if(Accel_roll < 100)
    {
      lcd.setCursor(12,2); 
      lcd.print(Accel_rollAbs);
      lcd.print(F(" "));
    }
    else 
    {
      lcd.setCursor(12,2); 
      lcd.print(Accel_rollAbs);
    }

    oldMag_roll = Accel_roll;
  }

  
  
}

void configure_magnetometer()
{
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);                            // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                                                 // Disable the I2C master interface
  Wire.endTransmission();
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);                  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                                 // Enable the bypass mux
  Wire.endTransmission();
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // CNTL[3:0] mode bits
  Wire.write(0b00011111);                                           // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                                      // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);                          // Request 3 bytes of data
  while (Wire.available() < 3);                                     // Wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;                       // Adjust data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00000000);                                           // Set mode to power down
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00010110);                                           // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change
}


void calibrate_magnetometer()
{
  int mag_x, mag_y, mag_z;
  int status_reg_2;                                               // ST2 status register
  int mag_x_min =  32767;                                         // Raw data extremes
  int mag_y_min =  32767;
  int mag_z_min =  32767;
  int mag_x_max = -32768;
  int mag_y_max = -32768;
  int mag_z_max = -32768;
  float chord_x,  chord_y,  chord_z;                              // Used for calculating scale factors
  float chord_average;
  lcd.clear();                                                    // Clear the LCD
  lcd.setCursor(0, 0);                                            // Set the LCD cursor to column 0 line 0
  lcd.print("Rotate Compass");                                    // Print text to screen
  lcd.setCursor(0, 1);                                            // Set the LCD cursor to column 0 line 1
  for (int counter = 0; counter < 16000 ; counter ++)             // Run this code 16000 times
  {
    Loop_start = micros();                                        // Start loop timer
    if (counter % 1000 == 0)lcd.print(".");                        // Print a dot on the LCD every 1000 readings

    Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
    Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
    while (Wire.available() < 1);                                 // Wait for the data
    if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
    {
      Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
      while (Wire.available() < 7);                               // Wait for the data
      mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
      status_reg_2 = Wire.read();                                 // Read status and signal data read
      if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
      {
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
    delay(4);                                                     // Time interval between magnetometer readings
  }
  Mag_x_offset = (mag_x_max + mag_x_min) / 2;                     // Get average magnetic bias in counts
  Mag_y_offset = (mag_y_max + mag_y_min) / 2;
  Mag_z_offset = (mag_z_max + mag_z_min) / 2;
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;                 // Get average max chord length in counts
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;
  chord_average = (chord_x + chord_y + chord_z) / 3;              // Calculate average chord length
  Mag_x_scale = chord_average / chord_x;                          // Calculate X scale factor
  Mag_y_scale = chord_average / chord_y;                          // Calculate Y scale factor
  Mag_z_scale = chord_average / chord_z;                          // Calculate Z scale factor
  if (Record_data == true)
  {
    Serial.print("XYZ Max/Min: ");
    Serial.print(mag_x_min); Serial.print("\t");
    Serial.print(mag_x_max); Serial.print("\t");
    Serial.print(mag_y_min); Serial.print("\t");
    Serial.print(mag_y_max); Serial.print("\t");
    Serial.print(mag_z_min); Serial.print("\t");
    Serial.println(mag_z_max);
    Serial.println("");
    Serial.print("Hard-iron: ");
    Serial.print(Mag_x_offset); Serial.print("\t");
    Serial.print(Mag_y_offset); Serial.print("\t");
    Serial.println(Mag_z_offset);
    Serial.println("");
    Serial.print("Soft-iron: ");
    Serial.print(Mag_x_scale); Serial.print("\t");
    Serial.print(Mag_y_scale); Serial.print("\t");
    Serial.println(Mag_z_scale);
    Serial.println("");
    Serial.print("ASA: ");
    Serial.print(ASAX); Serial.print("\t");
    Serial.print(ASAY); Serial.print("\t");
    Serial.println(ASAZ);
    while (true);                                       // Wheelspin ... program halt
  }
}


void read_magnetometer()
{
  int mag_x, mag_y, mag_z;
  int status_reg_2;
  Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
  Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
  while (Wire.available() < 1);                                 // Wait for the data
  if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
  {
    Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
    while (Wire.available() < 7);                               // Wait for the data
    mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read
    if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
    {
      Mag_x = (mag_x - Mag_x_offset) * Mag_x_scale;
      Mag_y = (mag_y - Mag_y_offset) * Mag_y_scale;
      Mag_z = (mag_z - Mag_z_offset) * Mag_z_scale;
    }
  }
}

void config_gyro()
{
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x6B);                                     //Point to power management register
  Wire.write(0x00);                                     //Use internal 20MHz clock
  Wire.endTransmission();                               //End the transmission
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1C);                                     //Point to accelerometer configuration reg
  Wire.write(0x10);                                     //Select +/-8g full-scale
  Wire.endTransmission();                               //End the transmission
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1B);                                     //Point to gyroscope configuration
  Wire.write(0x08);                                     //Select 500dps full-scale
  Wire.endTransmission();                               //End the transmission
}

void calibrate_gyro()
{
  lcd.clear();                                          //Clear the LCD
  lcd.setCursor(0, 0);                                  //Set the LCD cursor to position to position 0,0
  lcd.print("Calibrating gyro");                        //Print text to screen
  lcd.setCursor(0, 1);                                  //Set the LCD cursor to position to position 0,1
  pinMode(LED, OUTPUT);                                 //Set LED (pin 13) as output
  digitalWrite(LED, HIGH);                              //Turn LED on ... indicates startup
  for (int counter = 0; counter < 2000 ; counter ++)    //Run this code 2000 times
  {
    Loop_start = micros();
    if (counter % 125 == 0)lcd.print(".");              //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    Gyro_x_cal += Gyro_x;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    Gyro_y_cal += Gyro_y;                               //Add the gyro y-axis offset to the gyro_y_cal variable
    Gyro_z_cal += Gyro_z;                               //Add the gyro z-axis offset to the gyro_z_cal variable
    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  Gyro_x_cal /= 2000;                                   //Divide the gyro_x_cal variable by 2000 to get the average offset
  Gyro_y_cal /= 2000;                                   //Divide the gyro_y_cal variable by 2000 to get the average offset
  Gyro_z_cal /= 2000;                                   //Divide the gyro_z_cal variable by 2000 to get the average offset
}

void read_mpu_6050_data()
{
  int     temperature;                                  // Needed when reading the MPU-6050 data ... not used
  Wire.beginTransmission(0x68);                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                     // Point to start of data
  Wire.endTransmission();                               // End the transmission
  Wire.requestFrom(0x68, 14);                           // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                        // Wait until all the bytes are received
  Accel_x = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_x variable
  Accel_y = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_y variable
  Accel_z = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_z variable
  temperature = Wire.read() << 8 | Wire.read();         // Combine MSB,LSB temperature variable
  Gyro_x = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_y = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_z = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
}
