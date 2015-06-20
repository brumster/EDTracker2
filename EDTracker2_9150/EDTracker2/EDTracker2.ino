//
//  Head Tracker Sketch
//
const char  infoString []   = "EDTrackerMag V4.0.3";

//
// Changelog:
// 2014-11-10 RJ  Move to Arduino IDE 158 with newer copmiler = smaller code
//                Combine Invensense DMP output with continuous magnetometer
//                drift adjustment
// 2014-12-24 RJ  Implements startup auto calibration and new magnetometer drift compensation
// 2015-01-25 RJ  Fix mag wraping/clamping.
// 2015-02-13 RJ  Fix mag heading average when heading is close to +/- PI boundary
//            RJ  Fix incorrect roll compensation for mag heading
// 2015-03-01 RJ  Fix smoothing val not being saved to EEPROM
// 2015-06-20 DMH Fixed incorrect data type for sensor_data breaking compile; cannot upversion
//                due to lagging behind pocketmoon repo, but no functional change anyway
/* ============================================
EDTracker device code is placed under the MIT License

Copyright (c) 2014,2015 Rob James, Dan Howell

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


/* Starting sampling rate. Ignored if POLLMPU is defined above */
#define DEFAULT_MPU_HZ    (200)

#define EMPL_TARGET_ATMEGA328

#include <EEPROM.h>
#include <Wire.h>
#include <I2Cdev.h>

#include <helper_3dmath.h>
extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}


float xDriftComp = 0.0;

/* EEPROM Offsets for config and calibration stuff*/
#define EE_VERSION 0
// these are now longs (4 bytes)
#define EE_XGYRO 1
#define EE_YGYRO 5
#define EE_ZGYRO 9
#define EE_XACCEL 13
#define EE_YACCEL 17
#define EE_ZACCEL 21
// 1 byte
#define EE_ORIENTATION 25
// 2 bytes
#define EE_LPF 26

//0 for linear, 1 for exponential
#define EE_EXPSCALEMODE 28

//2x 1 byte  in 6:2   0.25 steps should be ok
#define EE_YAWSCALE 29
#define EE_PITCHSCALE 30
#define EE_YAWEXPSCALE 31
#define EE_PITCHEXPSCALE 32

//single bytes
#define EE_POLLMPU 33
#define EE_AUTOCENTRE 34

//The temp at which the stored drift compensation was measured.
//2 byte
#define EE_CALIBTEMP    35

//2 byte pairs in 9:7 format
#define EE_MAGOFFX      40
#define EE_MAGOFFY      42
#define EE_MAGOFFZ      44

//6 x 2 byte 2:14 format (signed)
#define EE_MAGXFORM     50

float magOffset[3];
float magXform[18];

#define SDA_PIN 2
#define SCL_PIN 3

#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BUTTON_PIN 10

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

boolean outputUI = false;

float lastDriftX;
int driftSamples = 0;

boolean expScaleMode = 0;
float scaleF[3];
float outputLPF = 0.5; //0 is no filter  max value 0.999...
unsigned char revision;

// packet structure for InvenSense teapot demo
unsigned long lastMillis = 0;
unsigned long lastUpdate = 0;

float c[4];

long gBias[3];

//Running count of samples - used when recalibrating
int   sampleCount = 0;
boolean calibrated = true;

//Number of samples to take when recalibrating
byte  recalibrateSamples =  255;

// Holds the time since sketch stared
unsigned long  nowMillis;
boolean blinkState;

float magHeading, lastMagHeading,   rawMagHeading;
;
int consecCount = 0;

boolean behind = false;
boolean offsetMag = false;
TrackState_t joySt;

/* The mounting matrix below tells the MPL how to rotate the raw
 * data from the driver(s).
 */

static byte gyro_orients[6] =
{
  B10001000, // right
  B10000101, // front
  B10101100, // left
  B10100001 // rear
}; //ZYX


byte orientation = 2;

//Need some helper funct to read/write integers
void writeIntEE(int address, int value) {
  EEPROM.write(address + 1, value >> 8); //upper byte
  EEPROM.write(address, value & 0xff); // write lower byte
}

int readIntEE(int address) {
  return (EEPROM.read(address + 1) << 8 | EEPROM.read(address));
}

void writeLongEE(int address,  long value) {
  for (int i = 0; i < 4; i++)
  {
    EEPROM.write(address++, value & 0xff); // write lower byte
    value = value >> 8;
  }
}

long readLongEE(int address) {
  return ((long)EEPROM.read(address + 3) << 24 |
          (long)EEPROM.read(address + 2) << 16 |
          (long)EEPROM.read(address + 1) << 8 |
          (long)EEPROM.read(address));
}
long avgGyro[3] ;//= {0, 0, 0};

void setup() {

  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // send a I2C stop signal
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);

  loadSettings();

  loadMagCalibration();

  // pass oreintated calib matrix back to UI
  //  reportRawMagMatrix();

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 12;//12; // 12 400kHz I2C clock (200kHz if CPU is 8MHz)

  // Disable internal I2C pull-ups
  cbi(PORTD, 0);
  cbi(PORTD, 1);

  // Initialize the MPU:
  initialize_mpu();
  enable_mpu();

}




/****************************************
* Gyro/Accel/DMP Configuration
****************************************/
#define PI    32768.0
#define TWOPI 65536.0

float wrap(float angle)
{
  if (angle > PI)
    angle -= (TWOPI);
  if (angle < -PI)
    angle += (TWOPI);
  return angle;
}

void recenter()
{
  if (outputUI)
  {
    Serial.println("M\tR");
  }
  sampleCount = 0;

  for (int i = 0; i < 4; i++)
    c[i] = 0.0;

  calibrated = false;
  offsetMag = false;

  if (rawMagHeading > 28000.0  || rawMagHeading < -28000.0)
    offsetMag = true;
  // Serial.println(rawMagHeading);
  //Serial.println(offsetMag);
}

volatile boolean new_gyro ;
boolean startup = true;
int  startupPhase = 0;
int  startupSamples;

#define PITCH 1
#define ROLL  2
#define YAW   0

void loop()
{
  long unsigned int sensor_data;
  short gyro[3], accel[3], sensors;
  unsigned char more ;
  unsigned char magSampled ;
  long quat[4];

  //Loop until MPU interrupts us with a reading
  while (!new_gyro)
    ;

  nowMillis = millis();

  sensor_data = 1;
  dmp_read_fifo(gyro, accel, quat, &sensor_data, &sensors, &more);

  if (!more)
    new_gyro = false;

  if (sensor_data == 0)
  {
    Quaternion q( (float)quat[0]  / 1073741824.0f,
                  (float)quat[1]  / 1073741824.0f,
                  (float)quat[2]  / 1073741824.0f,
                  (float)quat[3]  / 1073741824.0f);

    // Calculate Yaw/Pitch/Roll
    // Update client with yaw/pitch/roll and tilt-compensated magnetometer data

    float newv[3];
    //roll
    newv[2] =  atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);

    // pitch
    newv[1] = -asin(-2.0 * (q.x * q.z - q.w * q.y));

    //yaw
    newv[0] = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

    short mag[3];
    magSampled  = mpu_get_compass_reg(mag);

    if (magSampled == 0)
    {

      if (outputUI)
        reportRawMag(mag);

      float ox = (float)mag[0] - magOffset[0] ;
      float oy = (float)mag[1] - magOffset[1] ;
      float oz = (float)mag[2] - magOffset[2] ;

      float rx = ox * magXform[0] + oy * magXform[1] + oz * magXform[2];
      float ry = ox * magXform[3] + oy * magXform[4] + oz * magXform[5];
      float rz = ox * magXform[6] + oy * magXform[7] + oz * magXform[8];

      //  pitch/roll adjusted mag heading
      rawMagHeading = wrap(atan2(-(rz * sin(newv[PITCH]) - ry * cos(newv[PITCH])),
                                 rx * cos(newv[ROLL]) +
                                 ry * sin(-newv[ROLL]) * sin(newv[PITCH]) +
                                 rz * sin(-newv[ROLL]) * cos(newv[PITCH])) * -10430.06);

      magHeading = rawMagHeading;

      if (calibrated)
        magHeading = magHeading - c[3];

      if (offsetMag)
        magHeading = magHeading + 32768.0;

      magHeading = wrap(magHeading);

      // Auto Gyro Calibration
      if (startup)
      {
        blink();
        parseInput();

        if (startupPhase == 0)
        {
          for (int n = 0; n < 3; n++)
            gBias[n] = avgGyro[n] = 0;

          mpu_set_gyro_bias_reg(gBias);
          startupPhase = 1;
          startupSamples = 500;
          return;
        }

        if (startupPhase == 1)
        {
          if (startupSamples == 0)
          {
            for (int n = 0; n < 3; n++)
            {
              gBias[n] = (avgGyro[n] / -250);
            }
            xDriftComp = 0.0;
            mpu_set_gyro_bias_reg(gBias);
            startupPhase = 2;
            startupSamples = 0;
            lastDriftX = newv[0];
            //return;
          }
          else
          {
            for (int n = 0; n < 3; n++)
              avgGyro[n] += gyro[n];
          }
          startupSamples --;
          return ;
        }

        if (startupPhase == 2)
        {
          if (startupSamples == 1500)
          {
            xDriftComp = (newv[0] - lastDriftX) * 69.53373;
            startup = false;
            recenter();
            Serial.println("H"); // Hello
          }
          startupSamples++;
          return ;
        }
      }
    }

    //scale +/- PI to +/- 32767
    for (int n = 0; n < 3; n++)
      newv[n] = newv[n]   * 10430.06;

    // move to pre-calibrate
    if (!calibrated)
    {
      if (sampleCount < recalibrateSamples)
      { // accumulate samples
        for (int n = 0; n < 3; n++)
          c[n] += newv[n];

        c[3] += magHeading;
        sampleCount ++;
      }
      else
      {
        calibrated = true;
        for (int n = 0; n < 4; n++)
          c[n] = c[n] / (float)sampleCount;
        if (offsetMag)
          c[3] = c[3] - 32768.0;
        offsetMag = false;

        //dX = 0.0;//dY = dZ = 0.0;
        driftSamples = -2;
        recalibrateSamples = 100;//  calibration next time around
        if (outputUI )
        {
          sendInfo();
        }
      }
      return;
    }

    // apply drift offsets
    for (int n = 0; n < 3; n++)
      newv[n] = newv[n] - c[n];

    // this should take us back to zero BUT we may have wrapped so ..
    if (newv[0] < -32768.0)
      newv[0] += 65536.0;

    if (newv[0] > 32768.0)
      newv[0] -= 65536.0 ;

    long ii[3];

    for (int n = 0; n < 3; n++)
    {
      if (expScaleMode) {
        ii[n] = (long) (0.000122076 * newv[n] * newv[n] * scaleF[n]) * (newv[n] / abs(newv[n]));
      }
      else
      {
        // and scale to out target range plus a 'sensitivity' factor;
        ii[n] = (long)(newv[n] * scaleF[n] );
      }
    }

    // clamp after scaling to keep values within 16 bit range
    for (int n = 0; n < 3; n++)
      ii[n] = constrain(ii[n], -32767, 32767);

    // Do it to it.
    if (ii[0] > 30000  || ii[0] < -30000)
      joySt.xAxis = ii[0] ;
    else
      joySt.xAxis = joySt.xAxis * outputLPF + ii[0] * (1.0 - outputLPF) ;

    joySt.yAxis = joySt.yAxis * outputLPF + ii[1] * (1.0 - outputLPF) ;
    joySt.zAxis = joySt.zAxis * outputLPF + ii[2] * (1.0 - outputLPF) ;

    Tracker.setState(&joySt);

    // Have we been asked to recalibrate ?
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      recenter();
      // return;
    }

    //reports++;
    if (outputUI )
    {
      Serial.print(joySt.xAxis);
      printtab();
      Serial.print(joySt.yAxis);
      printtab();
      Serial.print(joySt.zAxis);
      printtab();

      tripple(accel);
      tripple(gyro);
      /*
              Serial.print((int)rawMagHeading ); //
              printtab();
              Serial.print((int)magHeading  );
              printtab();
              Serial.print((int)c[3] ); //
              printtab();
      */
      Serial.println("");

    }

    if (nowMillis > lastUpdate)
    {
      blink();

      // apply drift compensation
      c[0] = c[0] + xDriftComp;

      //handle wrap
      if (c[0] > 65536.0)
        c[0] = c[0] - 65536.0;
      else if (c[0] < -65536.0 )
        c[0] = c[0] + 65536.0;

      lastUpdate = nowMillis + 100;
      lastDriftX = newv[0];

      long tempNow;

      mpu_get_temperature (&tempNow);
      if (outputUI )
      {
        Serial.print("T\t");
        Serial.println(tempNow);
      }
    }

    // magHeading = constrain (magHeading, -32767, 32767);

    if (//magHeading != lastMagHeading  &&
      // abs(magHeading) < 10000  &&
      newv[1] < 6000.0  && newv[1] > -6000.0
    )
    {
      lastMagHeading = magHeading;

      // Mag is so noisy on 9150 we just ask 'is Mag ahead or behind DMP'
      // and keep a count of consecutive behinds or aheads and use this
      // to adjust our DMP heading and also tweak the DMP drift
      float delta = magHeading - newv[0];

      if (delta > 32768.0)
        delta = 65536.0 - delta;
      else if (delta < -32767)
        delta = 65536.0 + delta;

      if (delta > 0.0)
      {
        if (behind )
        {
          consecCount --;
        }
        else
        {
          consecCount = 0;
        }
        behind = true;
      }
      else
      {
        if (!behind)
        {
          consecCount  ++;
        }
        else
        {
          consecCount = 0;
        }
        behind = false;
      }

      // Tweek the yaw offset. 0.01 keeps the head central with no visible twitching
      c[0] = c[0] + (float)(consecCount * abs(consecCount)) * 0.012;

      //Also tweak the overall drift compensation.
      //DMP still suffers from 'warm-up' issues and this helps greatly.
      xDriftComp = xDriftComp + (float)(consecCount) * 0.00001;
    }
    parseInput();
  }

}


void parseInput()
{
  while (Serial.available() > 0)
  {
    // read the incoming byte:
    byte command = Serial.read();

    if (command == 'C')
    {
      int data;

      for (int i = 0; i < 2; i++)
      {
        data = Serial.read();
        data = data | Serial.read() << 8;
        scaleF[i] = (float)data / 256.0;
      }

      data = Serial.read();
      data = data | Serial.read() << 8;
      outputLPF = (float)data / 32767.0;
      Serial.print("M\t");
      Serial.println(outputLPF);

      setScales();
      scl();
    }
    else if (command == 'S')
    {
      outputUI = false;
      Serial.println("S"); //silent
      dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    }
    else if (command == 'H')
    {
      if (startup)
        Serial.println("h");
      else
        Serial.println("H"); // Hello
    }
    else if (command == 't')
    {
      //toggle linear/expoinential mode
      expScaleMode = !expScaleMode;
      getScales();
      EEPROM.write(EE_EXPSCALEMODE, expScaleMode);
      scl();
    }
    else if (command == 'V')
    {
      Serial.println("V"); //verbose
      sendInfo();
      scl();
      outputUI = true;
      // if (DEFAULT_MPU_HZ > 101)
      dmp_set_fifo_rate(DEFAULT_MPU_HZ / 2);
      reportRawMagMatrix();
    }
    else if (command == 'I')
    {
      sendInfo();
      scl();
      reportRawMagMatrix();
      sendByte('O', orientation);
    }
    else if (command == 'P')
    {
      mpu_set_dmp_state(0);
      orientation = (orientation + 1) % 4; //0 to 3
      dmp_set_orientation(gyro_orients[orientation]);
      mpu_set_dmp_state(1);
      sendByte('O', orientation);
      EEPROM.write(EE_ORIENTATION, orientation);
    }
    else if (command == 'R')
    {
      //recalibrate offsets
      recenter();
    }
    else if (command == 'r')
    {
      //20 second calibration
      fullCalib();
    }
    else if (command == 87) // full wipe
    {
      byte len, data;
      len = Serial.read();

      for (int i = 0; i < len; i++)
      {
        data = Serial.read();
        EEPROM.write( i, data );
      }

      loadSettings();
      loadMagCalibration();

      mpu_set_dmp_state(0);
      dmp_set_orientation(gyro_orients[orientation]);
      mpu_set_dmp_state(1);

    }
    else if (command == 36) // Mag Calibration Matrix pushed down from UI with orientation applied
    {
      byte data;
      for (int i = 0 ; i < 6; i++)
      {
        data = Serial.read();
        EEPROM.write(EE_MAGOFFX + i, data );
      }

      for (int i = 0 ; i < 36 ; i++) // 18 + 18 is raw mag xform then orientated xform
      {
        data = Serial.read();
        EEPROM.write(EE_MAGXFORM + i, data );
      }

      loadMagCalibration();
    }
  }
}


/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
//void gyro_data_ready_cb(void) {
//  new_gyro = 1;
//}

ISR(INT6_vect) {
  new_gyro = 1;
}


//void tap_cb (unsigned char p1, unsigned char p2)
//{
//  return;
//}


void initialize_mpu() {
  mpu_init(&revision);
  mpu_set_compass_sample_rate(100); // defaults to 100 in the libs

  /* Get/set hardware configuration. Start gyro. Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  //  mpu_set_gyro_fsr (2000);//250
  //  mpu_set_accel_fsr(2);//4

  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);

  dmp_load_motion_driver_firmware();
  dmp_set_orientation(gyro_orients[orientation]);

  //dmp_register_tap_cb(&tap_cb);

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_RAW_GYRO ;//| DMP_FEATURE_GYRO_CAL;

  //dmp_features = dmp_features |  DMP_FEATURE_TAP ;

  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);

  return ;
}

void disable_mpu() {
  mpu_set_dmp_state(0);
  EIMSK &= ~(1 << INT6);      //deactivate interupt
}

void enable_mpu() {
  EICRB |= (1 << ISC60) | (1 << ISC61); // sets the interrupt type for EICRB (INT6)
  EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6, etc
  mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
}

void blink()
{
  /**/
  if (nowMillis > lastMillis )
  {
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    if (startup)
      lastMillis = nowMillis + 100;
    else
      lastMillis = nowMillis + 500;
  }/**/
}


void tripple(short * v)
{
  for (int i = 0; i < 3; i++)
  {
    Serial.print(v[i] ); //
    printtab();
  }
}

void mess(char * m, long * v)
{
  Serial.print(m);
  Serial.print(v[0]); Serial.print(" / ");
  Serial.print(v[1]); Serial.print(" / ");
  Serial.println(v[2]);
}



void getScales()
{
  if (expScaleMode)
  {
    scaleF[0] = (float)EEPROM.read(EE_YAWEXPSCALE) / 4.0;
    scaleF[1] = (float)EEPROM.read(EE_PITCHEXPSCALE) / 4.0;
  }
  else
  {
    scaleF[0] = (float)EEPROM.read(EE_YAWSCALE) / 4.0;
    scaleF[1] = (float)EEPROM.read(EE_PITCHSCALE) / 4.0;
  }
  scaleF[2] = scaleF[1];
}


void setScales()
{
  if (expScaleMode)
  {
    EEPROM.write(EE_YAWEXPSCALE, (int)(scaleF[0] * 4));
    EEPROM.write(EE_PITCHEXPSCALE, (int)(scaleF[1] * 4));
  }
  else
  {
    EEPROM.write(EE_YAWSCALE, (int)(scaleF[0] * 4));
    EEPROM.write(EE_PITCHSCALE, (int)(scaleF[1] * 4));
  }
  scaleF[2] = scaleF[1];

  writeIntEE(EE_LPF, (int)(outputLPF * 32767.0));
}

void sendBool(char x, boolean v)
{
  Serial.print(x);
  printtab();
  Serial.println(v);
}

void sendByte(char x, byte b)
{
  Serial.print(x);
  printtab();
  Serial.println(b);
}

void sendInfo()
{
  Serial.print("I\t");
  Serial.println(infoString);
}

void
scl()
{
  Serial.print("s\t");
  Serial.print(expScaleMode);
  printtab();
  Serial.print(scaleF[0]);
  printtab();
  Serial.print(scaleF[1]);
  printtab();
  Serial.println(outputLPF);
}
void reportRawMag(short mag[])
{
  Serial.print("Q\t");
  for (int i = 0; i < 3; i++)
  {
    Serial.print(mag[i]);
    printtab();
  }
  Serial.println("");
}

void reportRawMagMatrix()
{
  Serial.print("q\t");
  for (int i = 0; i < 3 ; i++)
  {
    Serial.print( magOffset[i], 6);
    printtab();
  }
  //raw mag calibration
  for (int i = 9; i < 18 ; i++)
  {
    Serial.print( magXform[i], 13);
    printtab();
  }
  Serial.println("");
}

void fullCalib()
{
  recenter();
  startupPhase = 0;
  startup = true;
}

void printtab()
{
  Serial.print("\t");
}


void
loadMagCalibration()
{
  for (int i = 0; i < 3 ; i ++)
  {
    magOffset[i] = (float)readIntEE(EE_MAGOFFX + (i * 2)) / 64.0;
  }

  for (int i = 0; i < 18 ; i ++)//orientated then raw
  {
    magXform[i] = (float)readIntEE(EE_MAGXFORM + i * 2) / 8192.0;
  }
}


void
loadSettings()
{
  orientation = constrain(EEPROM.read(EE_ORIENTATION), 0, 3);
  expScaleMode = EEPROM.read(EE_EXPSCALEMODE);
  getScales();
  outputLPF = (float)readIntEE(EE_LPF) / 32767.0;
}

