/*Arduino code written version date(08/08/22) for use on Teensy4.1
   Code takes in 4 analog channel inputs and gives the option to save..
   data to built-in SD card or transmit via Serial.
   Authored by Sam Mahoney
   // edited to include IMU 9 axis using teensy, edits made in accordance with the example provided by Sparkfun MPU9250-Basic AHRS_I2C
*/
#include <SD.h>
#include <TimeLib.h>
//Using mpu9250-Sparkfun
#include "quaternionFilters.h"
#include "MPU9250.h"
//insert
#ifdef LCD
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
//
// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);
#endif // LCD
#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//define MPU9250_ADDRESS MPU9250_ADDRESS_AD1
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
//MPU9250_ADDRESS = 0x71;
//
//
//changed the switch statement
#define   SWITCH_POSITION_A_PIN   35 //changed from 19,18
#define   SWITCH_POSITION_B_PIN   36
#define   READ_SWITCH_POSITION_A  digitalRead(SWITCH_POSITION_A_PIN)
#define   READ_SWITCH_POSITION_B  digitalRead(SWITCH_POSITION_B_PIN)
//insert


// Variables
byte switchState = 2;             // to store switch reading 0 = off 1 = posA 2 = posB
byte lastSwitchState = 0;         // to check for change
int sample_time_delay = 300000; //1800000 is 30 mins
IntervalTimer myTimer; //Timer Interupt

File myFile; //File setup
String fileName = "test" ; //Filename

//Set Analog pins to read
const int Channel1 = A6;//A12
const int Channel2 = A7;//A13
const int Channel3 = A8; //A15
const int Channel4 = A9; //A14
const int SwitchPosA = 5;
const int SwitchPosB = 6;

//Set specific stop time for SD writing
//If < 10 use single digit ie 1st = 1

int STOPDAY = 0;
int STOPHOUR = 0;
int STOPMIN = 5;


//Initializing variables for countdown testing (Set to 0 when not testing)
float myValue1 = 0;
float myValue2 = 0;
float myValue3 = 0;
float myValue4 = 0;



//Defining SD card
const int sdChipSelect = BUILTIN_SDCARD;

/*TIME SYNC SETUP------------------------------------------------*/
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

//String fileNamestring = "test" ; //Filename
//String s2 = fileNamestring + day()+hour()+minute()+".csv";
//String fileName = s2;

/*SETUP FUNCTION-------------------------------------------*/
void setup() {
  // put your setup code here, to run once:

  pinMode(SWITCH_POSITION_A_PIN, INPUT);
  pinMode(SWITCH_POSITION_B_PIN, INPUT);
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  Wire.begin();// added
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  //If Teensy can sync with PC, let monitor know
  while (!Serial) {};

  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  // LCD // inserted

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

#ifdef LCD
  display.begin(); // Ini8ialize the display
  display.setContrast(58); // Set the contrast

  // Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0); display.print("MPU9250");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20, 40); display.print("60 ug LSB");
  display.display();
  delay(1000);

  // Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
#endif // LCD

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

#ifdef LCD
  display.setCursor(20, 0); display.print("MPU9250");
  display.setCursor(0, 10); display.print("I AM");
  display.setCursor(0, 20); display.print(c, HEX);
  display.setCursor(0, 30); display.print("I Should Be");
  display.setCursor(0, 40); display.print(0x71, HEX);
  display.display();
  delay(1000);

#endif // LCD // inserted
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
#ifdef LCD
    display.clearDisplay();

    display.setCursor(0, 0); display.print("MPU9250 bias");
    display.setCursor(0, 8); display.print(" x   y   z  ");

    display.setCursor(0,  16); display.print((int)(1000 * myIMU.accelBias[0]));
    display.setCursor(24, 16); display.print((int)(1000 * myIMU.accelBias[1]));
    display.setCursor(48, 16); display.print((int)(1000 * myIMU.accelBias[2]));
    display.setCursor(72, 16); display.print("mg");

    display.setCursor(0,  24); display.print(myIMU.gyroBias[0], 1);
    display.setCursor(24, 24); display.print(myIMU.gyroBias[1], 1);
    display.setCursor(48, 24); display.print(myIMU.gyroBias[2], 1);
    display.setCursor(66, 24); display.print("o/s");

    display.display();
    delay(1000);
#endif // LCD

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);
#ifdef LCD
    display.clearDisplay();
    display.setCursor(20, 0); display.print("AK8963");
    display.setCursor(0, 10); display.print("I AM");
    display.setCursor(0, 20); display.print(d, HEX);
    display.setCursor(0, 30); display.print("I Should Be");
    display.setCursor(0, 40); display.print(0x48, HEX);
    display.display();
    delay(1000);
#endif // LCD

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

#ifdef LCD
    display.clearDisplay();
    display.setCursor(20, 0);  display.print("AK8963");
    display.setCursor(0, 10);  display.print("ASAX ");
    display.setCursor(50, 10); display.print(myIMU.factoryMagCalibration[0], 2);
    display.setCursor(0, 20);  display.print("ASAY ");
    display.setCursor(50, 20); display.print(myIMU.factoryMagCalibration[1], 2);
    display.setCursor(0, 30);  display.print("ASAZ ");
    display.setCursor(50, 30); display.print(myIMU.factoryMagCalibration[2], 2);
    display.display();
    delay(1000);
#endif // LCD

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
#ifdef LCD
    display.clearDisplay();
    display.setCursor(20, 0); display.print("AK8963");
    display.setCursor(0, 10); display.print("ASAX "); display.setCursor(50, 10);
    display.print(myIMU.factoryMagCalibration[0], 2);
    display.setCursor(0, 20); display.print("ASAY "); display.setCursor(50, 20);
    display.print(myIMU.factoryMagCalibration[1], 2);
    display.setCursor(0, 30); display.print("ASAZ "); display.setCursor(50, 30);
    display.print(myIMU.factoryMagCalibration[2], 2);
    display.display();
    delay(1000);
#endif // LCD
  }
  if (c == 0x71) {
    Serial.print("functioning");
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }


  //inserted
  //Call SD card setup function
  initializeCard();

  //Check if fileName already eon the SD card
  if (SD.exists(fileName.c_str()));
  {
    SD.remove(fileName.c_str());
    Serial.println("\n File already exists. It has been deleted and will be overwritten\n");
  }

  // myTimer.begin(GetSignalValues, 10000); //Run GetSignalValues every 10000us
  fileName = fileName + day() + hour() + minute() + ".csv";
  //String fileName = s2;
  writeHeader(); //Opens file and Writes a header

}

/*MAIN LOOP-------------------------------------------------------*/
void loop() {
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
  /* if (READ_SWITCH_POSITION_A == HIGH) switchState = 1;
    if (READ_SWITCH_POSITION_B == HIGH) switchState = 2;
    else switchState = 0;

    switchState = 2; //Test switch states here   //taken out switch statement with comments
  *///insert of mpu9250
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  //
  // while(myValue1>0){ //Remove when constantly logging data.
  // while (1)(switchState == 2) {
  /*while(STOPDAY > day()&& STOPHOUR > hour() && STOPMIN > minute()){ //Include when defining specific monitoring times
    switch (switchState) {
    myTimer.begin(GetSignalValues, 10000); //Run GetSignalValues every 10000us /// inserted from 318
    //SD CARD SAVE
    case 1:
      SDCardSave();
      break;
    //SERIAL OUTPUT
    case 2:
      SerialTransmit();
      break;
    //DO NOTHING
    case 0:
      readMyFile();
      //Serial.println("Switch Off");
      break;
    }
    }*///prevents creeping timestamps
  while (1) {
    Serial.println("Input sample time: ");
    // Actuator id i.e define 1 = 0x141, velocity(check max), position
    while (!Serial.available()) {
    }; //remove this blocking function later, its just for a testing


    if (Serial.available()) {
      int throwaway = Serial.parseInt();
      //sample_time_delay = Serial.parseInt();
      Serial.println("Sampling time start: ");
      Serial.println(sample_time_delay);
      Serial.println("--end");
    }
    myFile = SD.open(fileName.c_str(), FILE_WRITE);
    myTimer.begin(GetSignalValues, 10000); //Run GetSignalValues every 10000us //changed from 10000
    delay(sample_time_delay);
    myTimer.end();
    Serial.println("Sampling time ended: ");
    Serial.println(sample_time_delay);
    Serial.println("--end");
    myFile.close();
    readMyFile();
  }


  //delay(1000);

}//Loop end

/*ANALOG CHANNEL READING------------------------------------------*/
void GetSignalValues() {
  myValue1 = analogRead(Channel1);
  myValue2 = analogRead(Channel2);
  myValue3 = analogRead(Channel3);
  myValue4 = analogRead(Channel4);
  //myValue1--; //Reduce myValue by 1
  //myValue2++;
  //myValue3--;
  //myValue4++;
  //insert header + save data

  myFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (myFile) {
    //Serial.println(myValue1);
    myFile.print(hour());
    myFile.print(':');
    if (minute() < 10)
      myFile.print('0');
    myFile.print(minute());
    myFile.print(':');
    if (second() < 10)
      myFile.print('0');
    myFile.print(second());

    //myFile.print(millis()); //Time since Arduino last reset in ms
    myFile.print(",");

    myFile.print(myValue1);
    myFile.print(",");

    myFile.print(myValue2);
    myFile.print(",");

    myFile.print(myValue3);
    myFile.print(",");

    myFile.print(myValue4);
    myFile.print(",");

    myFile.print(myIMU.ax);
    myFile.print(",");

    myFile.print(myIMU.ay);
    myFile.print(",");

    myFile.print(myIMU.az);
    myFile.print(",");

    myFile.print(myIMU.gx);
    myFile.print(",");

    myFile.print(myIMU.gy);
    myFile.print(",");

    myFile.print(myIMU.gz);
    myFile.print(",");

    myFile.print(myIMU.mx);
    myFile.print(",");

    myFile.print(myIMU.my);
    myFile.print(",");

    myFile.println(myIMU.mz);
    /* if want to place in need to add the printing of yaw pitch and roll

          myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                        * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                        * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                        * *(getQ()+3));
          myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                        * *(getQ()+2)));
          myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                        * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                        * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                        * *(getQ()+3));
          myIMU.pitch *= RAD_TO_DEG;
          myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
          //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
          // - http://www.ngdc.noaa.gov/geomag-web/#declination
          myIMU.yaw  -= 8.5;
          myIMU.roll *= RAD_TO_DEG;
    */
    //  myFile.close();
  }
}
/*TRANSMIT TO SERIAL----------------------------------------------*/
void SerialTransmit() {
  Serial.print(hour());
  Serial.print(':');
  if (minute() < 10)
    Serial.print('0');
  Serial.print(minute());
  Serial.print(':');
  if (second() < 10)
    Serial.print('0');
  Serial.print(second());

  Serial.print(",");

  Serial.print(myValue1);
  Serial.print(",");

  Serial.print(myValue2);
  Serial.print(",");

  Serial.print(myValue3);
  Serial.print(",");

  Serial.print(myValue4);
  Serial.print(",");

  Serial.print(myIMU.ax);
  Serial.print(",");

  Serial.print(myIMU.ay);
  Serial.print(",");

  Serial.print(myIMU.az);
  Serial.print(",");

  Serial.print(myIMU.gx);
  Serial.print(",");

  Serial.print(myIMU.gy);
  Serial.print(",");

  Serial.print(myIMU.gz);
  Serial.print(",");

  Serial.print(myIMU.mx);
  Serial.print(",");

  Serial.print(myIMU.my);
  Serial.print(",");

  Serial.println(myIMU.mz);

  //inserted
  delay(10);
}

/*WRITE THE SD FILE HEADER----------------------------------------*/
void writeHeader() {
  String s2 = fileName + day() + hour() + minute() + ".csv";

  //String fileName = s2+day()+hour()+minute()+".csv";;
  myFile = SD.open(fileName.c_str(), FILE_WRITE);

  if (myFile) {
    Serial.println("---------------------------------\n");
    Serial.print("Writing to " + fileName + ": ");

    myFile.println("Time,Channel1,Channel2,Channel3,Channel4,myIMU.ax,myIMU.ay,myIMU.az,myIMU.gx,myIMU.gy,myIMU.gz,myIMU.mx,myIMU.my,myIMU.mz");

    myFile.close();
  }
  else
  {
    Serial.println("error opening" + fileName);
  }

}
/*SAVE DATA TO SD FILE-------------------------------*/
void SDCardSave() {
  myFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (myFile) {
    Serial.println(myValue1);
    myFile.print(hour());
    myFile.print(':');
    if (minute() < 10)
      myFile.print('0');
    myFile.print(minute());
    myFile.print(':');
    if (second() < 10)
      myFile.print('0');
    myFile.print(second());

    //myFile.print(millis()); //Time since Arduino last reset in ms
    myFile.print(",");

    myFile.print(myValue1);
    myFile.print(",");

    myFile.print(myValue2);
    myFile.print(",");

    myFile.print(myValue3);
    myFile.print(",");

    myFile.print(myValue4);
    myFile.print(",");

    myFile.print(myIMU.ax);
    myFile.print(",");

    myFile.print(myIMU.ay);
    myFile.print(",");

    myFile.print(myIMU.az);
    myFile.print(",");

    myFile.print(myIMU.gx);
    myFile.print(",");

    myFile.print(myIMU.gy);
    myFile.print(",");

    myFile.print(myIMU.gz);
    myFile.print(",");

    myFile.print(myIMU.mx);
    myFile.print(",");

    myFile.print(myIMU.my);
    myFile.print(",");

    myFile.println(myIMU.mz);
    /* if want to place in need to add the printing of yaw pitch and roll

          myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                        * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                        * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                        * *(getQ()+3));
          myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                        * *(getQ()+2)));
          myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                        * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                        * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                        * *(getQ()+3));
          myIMU.pitch *= RAD_TO_DEG;
          myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
          //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
          // - http://www.ngdc.noaa.gov/geomag-web/#declination
          myIMU.yaw  -= 8.5;
          myIMU.roll *= RAD_TO_DEG;
    */
    myFile.close();
    delay(100);
  }
  else {
    Serial.println("error opening " + fileName);
  }

}
/*INITIALIZE THE SD FILE------------------------------------------*/
void initializeCard()
{
  Serial.print("Beginning SD Card initialization: ");

  if (!SD.begin(sdChipSelect))
  {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");
  Serial.println("------------------------------\n");
}
//End initialzeCard()

/*DOUBLE DIGITS FOR TIME PRINT----------------------------------*/

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
//End printDigits()

/*PROCESS TIME SYNC SERIAL CONNECTION----------------------------*/
/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

/*READ SD FILE----------------------------------------------------*/
void readMyFile() {
  myFile = SD.open(fileName.c_str());

  if (myFile)
  {
    Serial.println("-------------------------\n");
    Serial.println("Reading data stored in: " + fileName + ":");

    while (myFile.available()) //While there is unread data in the file
    {
      Serial.write(myFile.read()); //Read data and write to Serial
      //delay(1);
    }
    myFile.close();
  }
  else {
    Serial.println("error opening " + fileName);
  }

}
/*END FILE----------------------------------------------------*/
