
#include "SPI.h"
#include "Wire.h"
#include "I2Cdev.h"

#include "ZETA1.h"
#include "Pegasus_CtrlData.h"
#include "Pegasus_Log.h"

#include "MPU9255.h"
#include "helper_3dmath.h"

#include "GyroAngle.h"
#include "AccelAngle.h"
#include "CompassAngle.h"
#include "XRotor.h"
#include "pPID.h"
#include "CurveRemap.h"

//#define DEBUG
#ifdef DEBUG
#define Serial_begin(x) Serial.begin(x);
#define print(x) Serial.print(x);
#else
#define Serial_begin(x);
#define print(x);
#endif

// status lights
const byte statPin_r = 2;
const byte statPin_y = 3;
const byte statPin_g = 4;

byte buf[64]; // general use buffer

// RX
ZETA1 radio(10, 8, 9); // CS, INT, SDN pins
Pegasus_CtrlData ctrl;
byte validPacketCount = 0;
byte connectionLostCount = 0;

// IMU
MPU9255 imu;
GyroAngle gyro;
AccelAngle accel;
CompassAngle compass;
// raw sensor readings
short ax, ay, az, gx, gy, gz, mx, my, mz;

// Logging
Pegasus_Log logger;
const byte logPin = 15;

// Motor Power Control
XRotor esc;
const byte motorPin_FL = 20;
const byte motorPin_FR = 22;
const byte motorPin_BR = 23;
const byte motorPin_BL = 21;
unsigned short throttleMax = 65535;

pPID PID_yaw;
pPID PID_pitch;
pPID PID_roll;
// PID output
long op_roll, op_pitch, op_yaw, op_throttle;
// motor power output
long power_FL, power_FR, power_BR, power_BL;

// throttle-thrust correction
CurveRemap tt;

#define FLTTLENGTH 2
unsigned short FL_tt_x[FLTTLENGTH] = {0,65535};
unsigned short FL_tt_y[FLTTLENGTH] = {3195,65535};
#define FRTTLENGTH 2
unsigned short FR_tt_x[FRTTLENGTH] = {0,65535};
unsigned short FR_tt_y[FRTTLENGTH] = {3195,65535};
#define BRTTLENGTH 2
unsigned short BR_tt_x[BRTTLENGTH] = {0,65535};
unsigned short BR_tt_y[BRTTLENGTH] = {3195,65535};
#define BLTTLENGTH 2
unsigned short BL_tt_x[BLTTLENGTH] = {0,65535};
unsigned short BL_tt_y[BLTTLENGTH] = {3195,65535};
/*
#define FLTTLENGTH 14
unsigned short FL_tt_x[FLTTLENGTH] = {0,1843,25384,25465,26711,32691,37411,44993,45244,47404,53389,60687,62966,65535};
unsigned short FL_tt_y[FLTTLENGTH] = {3359,4022,29892,30626,31276,37814,41815,47131,47705,48586,52278,56283,58021,59281};
#define FRTTLENGTH 20
unsigned short FR_tt_x[FRTTLENGTH] = {0,2268,2696,4090,5315,7321,8233,13482,15232,18479,19297,20591,22920,27050,36909,42039,44703,46391,50929,51161};
unsigned short FR_tt_y[FRTTLENGTH] = {3333,6638,8364,10341,14272,17878,22184,27972,31221,33957,35476,36489,37444,40967,50382,54623,57979,58616,63501,65535};
#define BRTTLENGTH 12
unsigned short BR_tt_x[BRTTLENGTH] = {0,2225,25679,25748,27688,36836,41243,44899,45651,47255,63428,65535};
unsigned short BR_tt_y[BRTTLENGTH] = {3277,4077,28358,29310,30467,40393,43836,46233,47092,47810,56777,57890};
#define BLTTLENGTH 11
unsigned short BL_tt_x[BLTTLENGTH] = {0,2558,25426,26377,26494,28842,41066,47144,47512,49400,65535};
unsigned short BL_tt_y[BLTTLENGTH] = {3195,3894,24949,26229,26919,28269,40086,44674,45374,46146,55350};
*/
// timers
unsigned long elapsed_ms;

unsigned short longPeriod = 500;
unsigned long lastLongTime = millis();

byte loggerPeriod = 40;
unsigned long lastloggerTime = millis();

byte midPeriod = 21;
unsigned long lastMidTime = millis();

unsigned long elapsed_si;
unsigned short shortPeriod = 5000; //microseconds(si) 2500=400Hz, 5000=200Hz, 10000=100Hz
unsigned long lastShortTime = micros();
unsigned long lastShortTimeMS = millis();

void setup() {
    pinMode(statPin_r, OUTPUT);
    pinMode(statPin_y, OUTPUT);
    pinMode(statPin_g, OUTPUT);
    digitalWrite(statPin_r, LOW);
    digitalWrite(statPin_y, LOW);
    digitalWrite(statPin_g, LOW);
    pinMode(logPin, INPUT_PULLUP);
    delayMicroseconds(10);

    Wire.begin();
    //Wire.setClock(400000L);

    // logging, attach jumper to enter read log mode
    logger.init();
    if(!digitalRead(logPin)){
        Serial.begin(115200);
        while (!Serial);
        logger.readLog();
        while(true);
    }

    Serial_begin(115200);
    print("Enter Setup\n");
    digitalWrite(statPin_y, HIGH);

    // ESC initialize
    print("ESC Setup...");
    esc.config(0, throttleMax, 480, 16);// inputMin, inputMax, frequency, resolution
    esc.init(motorPin_FL);
    esc.init(motorPin_FR);
    esc.init(motorPin_BR);
    esc.init(motorPin_BL);
    esc_calibrate();
    print("ESC Setup Complete\n");

    // RX initialize
    print("RX Setup...");
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    radio.reset();
    if ( radio.initialize() != radio.SI4455_SUCCESS ) {
        print("Radio_Init_Failed\n");
        digitalWrite(statPin_y, LOW);
        digitalWrite(statPin_r, HIGH);
        while (true) {}
    }
    radio.startRX();
    print("RX Setup Complete\n");

    pid_config();

    //GyroAngle.config( ushort FullScaleRangeDPS,
    //                  ulong deltaTime_UnitsPerSecond,
    //                  float correction_weight
    //                  bool invX, invY, invZ)
    gyro.config(2000, 1000000, 0.01, false, true, true);
    accel.config(false, true, false);//args are for inverting axis
    compass.config(152.0, 197.5, -172.5, 1.076132, 1.039761, 1.0); // see compass_calibrate()
  
    // IMU initialize
    print("IMU Setup...");
    imu.initialize();
    imu.setFullScaleGyroRange(MPU9255_GYRO_FS_2000);//250, 500, 1000, 2000dps
    imu.setFullScaleAccelRange(MPU9255_ACCEL_FS_4);//2, 4, 8, 16g
    imu.magInitialize();
    if (!imu.testConnection()) {
        print("Connection_Failed\n");
        digitalWrite(statPin_y, LOW);
        digitalWrite(statPin_r, HIGH);
        while (true) {}
    }
    print("IMU Setup Complete\n");
    imu_calibrate();    
    //compass_calibrate();
    
    // clear log
    //print("Clearing Log...");
    //logger.clear();
    //print("Clear\n");
    
    print("Setup Complete\n");

    /*
    print("Waiting on RX...");
    while(ctrl.control_code != ctrl.NAV_CODE){
        if (millis() - lastMidTime > midPeriod) {
            lastMidTime = millis();
            rx_check();
        }
    }
    print("Connected\n");
    */
    digitalWrite(statPin_y, LOW);
    digitalWrite(statPin_g, HIGH);

    // reset timers
    lastLongTime = millis();
    lastloggerTime = millis();
    lastMidTime = millis();
    lastShortTime = micros();
    lastShortTimeMS = millis();
}

void loop() {
  elapsed_ms = millis() - lastLongTime;
  if (elapsed_ms >= longPeriod) {
    lastLongTime = millis();
    rx_watchdog();
  }

  elapsed_ms = millis() - lastMidTime;
  if (elapsed_ms >= midPeriod) {
    lastMidTime = millis();
    if (rx_check()) {
      validPacketCount++;
    }
  }

  elapsed_si = micros() - lastShortTime;
  if (elapsed_si >= shortPeriod) {
    lastShortTime = micros();
    lastShortTimeMS = millis();
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //bool updatedMag = imu.getMagReading( &mx, &my, &mz);
    gyro.update(gx, gy, gz, elapsed_si);
    //print("Rate "); print(lastShortTimeMS); print(" "); print(gyro.rateX); print(" "); print(gyro.rateY); print(" "); print(gyro.rateZ); print("\n");
    //print("Gyro "); print(lastShortTimeMS); print(" "); print(gyro.accX); print(" "); print(gyro.accY); print(" "); print(gyro.accZ); print("\n");
    accel.update(ax, ay, az);
    //print("Accel "); print(lastShortTimeMS); print(" "); print(accel.x); print(" "); print(accel.y); print(" "); print(accel.z); print("\n");
    gyro.correctXY(accel.x, accel.y);
    /*
    if(updatedMag){ 
        compass.update(mx, my, mz, ax, ay, az);
        //print("Compass "); print(lastShortTimeMS); print(" ");print(compass.mx); print(" "); print(compass.my); print(" "); print(compass.mz); print("\n");
        gyro.correctZ(compass.yaw); 
    }
    */
    //print("IMU "); print(lastShortTimeMS); print(" "); print(gyro.posX); print(" "); print(gyro.posY); print(" "); print(gyro.posZ); print("\n");
    if (ctrl.live) {
        pid_update();
        motor_update();
    }else{
        motor_off();
    }
  }

  elapsed_ms = millis() - lastloggerTime;
  if (elapsed_ms >= loggerPeriod) {
    lastloggerTime = millis();
    /*if(ctrl.live){
        logger.append_IMU(lastloggerTime, gyro.posX, gyro.posY, gyro.posZ);
        logger.append_PID(lastloggerTime, op_roll, op_pitch, op_yaw);
        logger.append_MP(lastloggerTime, power_FL, power_FR, power_BR, power_BL);
        logger.append_RX(lastloggerTime, ctrl.rollf, ctrl.pitchf, ctrl.yawf, ctrl.throttle);
        logger.writeOnFull();        
    }*/
    print("IMU "); print(lastShortTimeMS); print(" "); print(gyro.posX); print(" "); print(gyro.posY); print(" "); print(gyro.posZ); print("\n");
    print("PID "); print(lastShortTimeMS); print(" "); print(op_roll); print(" "); print(op_pitch); print(" "); print(op_yaw); print("\n");
    print("MP "); print(lastShortTimeMS); print(" "); print(power_FL); print(" "); print(power_FR); print(" "); print(power_BR); print(" "); print(power_BL); print("\n");
  }
}

bool rx_check() {
  byte packetCount = radio.checkReceived(buf); // check for new rx data
  if (packetCount) {
    // get last packet received
    byte startIndex = (packetCount - 1) * radio.packetLength;
    // move latest packet to front
    for (byte i = 0; i < radio.packetLength; i++) {
      buf[i] = buf[i + startIndex];
    }
    ctrl.unpack(buf);
    switch (ctrl.control_code) {
      case ctrl.KILL_CODE:
        //print("Kill\n");
        return true;
      case ctrl.NAV_CODE: // navigation control packet
        print("Nav "); print(lastMidTime); print(" "); print(ctrl.rollf); print(" "); print(ctrl.pitchf); print(" "); print(ctrl.yawf); print(" "); print(ctrl.throttle); print("\n");
        return true;
      case ctrl.NAV_ROT_MODE_CODE: // navigation rotation mode; Stable, Acrobatic
        print("Nav_Rot_Mode "); print(ctrl.navRotModeRoll); print(" "); print(ctrl.navRotModePitch); print(" "); print(ctrl.navRotModeYaw); print("\n");
        return true;
      case ctrl.NAV_ROT_RANGE_CODE: // navigation rotation range
        print("Nav_Rot_Range "); print(ctrl.navRotRangeRoll); print(" "); print(ctrl.navRotRangePitch); print(" "); print(ctrl.navRotRangeYaw); print("\n");
        return true;
      case ctrl.PID_YAW_CODE: // PID Yaw settings
        print("Pid_Yaw_Settings "); print(ctrl.Kp_yaw); print(" "); print(ctrl.Ki_yaw); print(" "); print(ctrl.Kd_yaw); print("\n");
        pid_config();
        return true;
      case ctrl.PID_PITCH_CODE: // PID Pitch settings
        print("Pid_Pitch_Settings "); print(ctrl.Kp_pitch); print(" "); print(ctrl.Ki_pitch); print(" "); print(ctrl.Kd_pitch); print("\n");
        pid_config();
        return true;
      case ctrl.PID_ROLL_CODE: // PID Roll settings
        print("Pid_Roll_Settings "); print(ctrl.Kp_roll); print(" "); print(ctrl.Ki_roll); print(" "); print(ctrl.Kd_roll); print("\n");
        pid_config();
        return true;
      default: // unknown control code
        print("UNKNOWN ");
        for (byte i = 0; i < radio.packetLength; i++) {
          print(buf[i]);
          print(" ");
        } print("\n");
        return false;
        break;
    }
  }
  return false;
}

void rx_watchdog() {
  // checked every 500ms
  if (validPacketCount < 10) { // should be ~22 per 500ms
    // set nav control to hover
    ctrl.yawf = 0.0;
    ctrl.pitchf = 0.0;
    ctrl.rollf = 0.0;
    ctrl.throttle = 0.0;
    connectionLostCount++;
    digitalWrite(statPin_g, LOW);
    digitalWrite(statPin_r, HIGH);
    print("Got fewer packets then expected\n");
  } else {
    connectionLostCount = 0;
    digitalWrite(statPin_g, HIGH);
    digitalWrite(statPin_r, LOW);
  }
  validPacketCount = 0;
  if(connectionLostCount > 6){ // shut down after 3 seconds of lost connection
    ctrl.live = false;
    print("Lost connection, shutdown\n");
  }
}

void pid_update() {
  // update PID & scale to power
  //print("PID_R "); print(lastShortTimeMS); print(" "); print(ctrl.rollf); print(" ");
  switch (ctrl.navRotModeRoll) {
    case ctrl.FLY_STAB:
      op_roll = long(PID_roll.compute( ctrl.rollf, gyro.posX));
      //print(gyro.posX); print(" ");
      break;
    case ctrl.FLY_ACRO:
      op_roll = long(PID_roll.compute( ctrl.rollf, gyro.rateX));
      //print(gyro.rateX); print(" ");
      break;
  }
  //print(PID_roll.getError()); print(" "); print(ctrl.Kp_roll); print(" "); print(ctrl.Kp_roll * PID_roll.getError()); print(" ")
  //print(PID_roll.getIntegral()); print(" "); print(ctrl.Ki_roll); print(" "); print(ctrl.Ki_roll * PID_roll.getIntegral()); print(" ")
  //print(PID_roll.getDerivative()); print(" "); print(ctrl.Kd_roll); print(" "); print(ctrl.Kd_roll * PID_roll.getDerivative()); print(" ")
  //print(op_roll); print("\n");

  //print("PID_P "); print(lastShortTimeMS); print(" "); print(ctrl.pitchf); print(" ");
  switch (ctrl.navRotModePitch) {
    case ctrl.FLY_STAB:
      op_pitch = long(PID_pitch.compute( ctrl.pitchf, gyro.posY));
      //print(gyro.posY); print(" ");
      break;
    case ctrl.FLY_ACRO:
      op_pitch = long(PID_pitch.compute( ctrl.pitchf, gyro.rateY));
      //print(gyro.rateY); print(" ");
      break;
  }
  //print(PID_pitch.getError()); print(" "); print(ctrl.Kp_pitch); print(" "); print(ctrl.Kp_pitch * PID_pitch.getError()); print(" ")
  //print(PID_pitch.getIntegral()); print(" "); print(ctrl.Ki_pitch); print(" "); print(ctrl.Ki_pitch * PID_pitch.getIntegral()); print(" ")
  //print(PID_pitch.getDerivative()); print(" "); print(ctrl.Kd_pitch); print(" "); print(ctrl.Kd_pitch * PID_pitch.getDerivative()); print(" ")
  //print(op_pitch); print("\n");

  //print("PID_Y "); print(lastShortTimeMS); print(" "); print(ctrl.yawf); print(" ");
  switch (ctrl.navRotModeYaw) {
    case ctrl.FLY_STAB: // use angle position for "stable" mode
      op_yaw = long(PID_yaw.compute( ctrl.yawf, gyro.posZ));
      //print(gyro.posZ); print(" ");
      break;
    case ctrl.FLY_ACRO: // use angle rate for "acrobatic" mode
      op_yaw = long(PID_yaw.compute( ctrl.yawf, gyro.rateZ));
      //print(gyro.rateZ); print(" ");
      break;
  }
  //print(PID_yaw.getError()); print(" "); print(ctrl.Kp_yaw); print(" "); print(ctrl.Kp_yaw * PID_yaw.getError()); print(" ")
  //print(PID_yaw.getIntegral()); print(" "); print(ctrl.Ki_yaw); print(" "); print(ctrl.Ki_yaw * PID_yaw.getIntegral()); print(" ")
  //print(PID_yaw.getDerivative()); print(" "); print(ctrl.Kd_yaw); print(" "); print(ctrl.Kd_yaw * PID_yaw.getDerivative()); print(" ")
  //print(op_yaw); print("\n");
}

void pid_config() {
  // pid.config(float p, float i, float d, float outputMin, float outputMax)
  PID_yaw.config(shortPeriod, ctrl.Kp_yaw, ctrl.Ki_yaw, ctrl.Kd_yaw, -65535.0, 65535.0);
  PID_pitch.config(shortPeriod, ctrl.Kp_pitch, ctrl.Ki_pitch, ctrl.Kd_pitch, -65535.0, 65535.0);
  PID_roll.config(shortPeriod, ctrl.Kp_roll, ctrl.Ki_roll, ctrl.Kd_roll, -65535.0, 65535.0);
}

void motor_update() {  
  //    roll
  // fl-----fr
  // |  yaw  | pitch
  // bl-----br

  op_throttle = ctrl.throttle;
  power_FL = op_throttle + op_pitch + op_roll - op_yaw;
  power_FR = op_throttle + op_pitch - op_roll + op_yaw;
  power_BR = op_throttle - op_pitch - op_roll - op_yaw;
  power_BL = op_throttle - op_pitch + op_roll + op_yaw;
  /*
  power_FL = tt.interp(power_FL, FL_tt_x, FL_tt_y, FLTTLENGTH);
  power_FR = tt.interp(power_FR, FR_tt_x, FR_tt_y, FRTTLENGTH);
  power_BR = tt.interp(power_BR, BR_tt_x, BR_tt_y, BRTTLENGTH);
  power_BL = tt.interp(power_BL, BL_tt_x, BL_tt_y, BLTTLENGTH);
  */
  
  //print("MP "); print(lastShortTimeMS); print(" "); print(power_FL); print(" "); print(power_FR); print(" "); print(power_BR); print(" "); print(power_BL); print("\n");

  esc.update(motorPin_FL, power_FL);
  esc.update(motorPin_FR, power_FR);
  esc.update(motorPin_BR, power_BR);
  esc.update(motorPin_BL, power_BL);
}

void motor_off() {
    esc.update(motorPin_FL, 0);
    esc.update(motorPin_FR, 0);
    esc.update(motorPin_BR, 0);
    esc.update(motorPin_BL, 0);
}

void imu_calibrate() {
  print("IMU Calibration...\n");
  print("Sensor should be at rest\n");
  // gyro offset values
  long gx_sum = 0; long gy_sum = 0; long gz_sum = 0; // sum
  float gx_avg = 0; float gy_avg = 0; float gz_avg = 0; // average
  float gx_pavg = 0; float gy_pavg = 0; float gz_pavg = 0; // previous average

  // vector from accelerometer
  long ax_sum = 0; long ay_sum = 0; long az_sum = 0; // sum
  float ax_avg = 0; float ay_avg = 0; float az_avg = 0; // average
  float ax_pavg = 0; float ay_pavg = 0; float az_pavg = 0; // previous average
  
  // vector from compass
  long mx_sum = 0; long my_sum = 0; long mz_sum = 0; // sum
  float mx_avg = 0; float my_avg = 0; float mz_avg = 0; // average
  float mx_pavg = 0; float my_pavg = 0; float mz_pavg = 0; // previous average

  bool stable = false;
  float g_epsilon = 0.01;
  float a_epsilon = 0.1;
  float m_epsilon = 0.01;
  float count = 0.0;
  do {
    stable = true;
    count += 1.0;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    ax_sum += ax; ay_sum += ay; az_sum += az;
    mx_sum += mx; my_sum += my; mz_sum += mz;
    gx_avg = gx_sum / count; gy_avg = gy_sum / count; gz_avg = gz_sum / count;
    ax_avg = ax_sum / count; ay_avg = ay_sum / count; az_avg = az_sum / count;
    mx_avg = mx_sum / count; my_avg = my_sum / count; mz_avg = mz_sum / count;
    if(int(count) > 20){
        if(fabs(gx_avg-gx_pavg) > g_epsilon){ stable = false;}
        if(fabs(gy_avg-gy_pavg) > g_epsilon){ stable = false;}
        if(fabs(gz_avg-gz_pavg) > g_epsilon){ stable = false;}
        if(fabs(ax_avg-ax_pavg) > a_epsilon){ stable = false;}
        if(fabs(ay_avg-ay_pavg) > a_epsilon){ stable = false;}
        if(fabs(az_avg-az_pavg) > a_epsilon){ stable = false;}
        if(fabs(mx_avg-mx_pavg) > m_epsilon){ stable = false;}
        if(fabs(my_avg-my_pavg) > m_epsilon){ stable = false;}
        if(fabs(mz_avg-mz_pavg) > m_epsilon){ stable = false;}
    }else{
        stable = false;
    }
    gx_pavg = gx_avg; gy_pavg = gy_avg; gz_pavg = gz_avg;
    ax_pavg = ax_avg; ay_pavg = ay_avg; az_pavg = az_avg;
    mx_pavg = mx_avg; my_pavg = my_avg; mz_pavg = mz_avg;
    delay(5);
  }while(!stable);
  
  compass.update(int(mx_avg), int(my_avg), int(mz_avg), int(ax_avg), int(ay_avg), int(az_avg));
  compass.init(compass.yaw);
  accel.setRestingPosition(0.0, 0.0, 1.0);
  //accel.setRestingPosition(int(ax_avg), int(ay_avg), int(az_avg)); // only use if craft is perfectly level
  accel.update(int(ax_avg), int(ay_avg), int(az_avg));
  gyro.init(gx_avg, gy_avg, gz_avg, accel.x, accel.y, 0.0); // gyro offset xyz, accel starting rotation xyz
  #ifdef DEBUG
    Serial.printf("accel resting vector %0.3f %0.3f %0.3f\n", ax_avg, ay_avg, az_avg);
    Serial.printf("accel current angle %0.3f %0.3f %0.3f\n", accel.x, accel.y, accel.z);
    Serial.printf("gyro rate offset %0.3f %0.3f %0.3f\n", gx_avg, gy_avg, gz_avg);
    Serial.printf("compass resting heading %0.3f\n", compass.yaw);
  #endif
  print("IMU Calibration Complete\n");
}

void compass_calibrate(){
  print("Compass Calibration...\n");
  print("move sensor around in all orientations, monitor the min/max values, run until satisfied\n");
  print("use results to set compass.config(Xoffset,Yoffset,Zoffset,Xscale,Yscale,Zscale);");
  delay(2000);
  float Xoffset, Yoffset, Zoffset;
  float Xscale, Yscale, Zscale;
  float longCord;
  int16_t Xmax = -32760; 
  int16_t Xmin = 32760;
  int16_t Ymax = -32760; 
  int16_t Ymin = 32760;
  int16_t Zmax = -32760; 
  int16_t Zmin = 32760;
  while(true){
    while(!imu.getMagReading( &mx, &my, &mz)){}
    Xmax = mx > Xmax ? mx : Xmax;
    Xmin = mx < Xmin ? mx : Xmin;
    Ymax = my > Ymax ? my : Ymax;
    Ymin = my < Ymin ? my : Ymin;
    Zmax = mz > Zmax ? mz : Zmax;
    Zmin = mz < Zmin ? mz : Zmin;
    
    Xoffset = (Xmax + Xmin)/2.0;
    Yoffset = (Ymax + Ymin)/2.0;
    Zoffset = (Zmax + Zmin)/2.0;

    // find scale from largest chord length
    longCord = float(max( max(Xmax - Xmin, Ymax - Ymin), Zmax - Zmin));
    Xscale = longCord/(Xmax - Xmin);
    Yscale = longCord/(Ymax - Ymin);
    Zscale = longCord/(Zmax - Zmin);

    #ifdef DEBUG
        Serial.printf("X min %i max %i = offset %0.6f scale %0.6f\n", Xmin, Xmax, Xoffset, Xscale);
        Serial.printf("Y min %i max %i = offset %0.6f scale %0.6f\n", Ymin, Ymax, Yoffset, Yscale);
        Serial.printf("Z min %i max %i = offset %0.6f scale %0.6f\n", Zmin, Zmax, Zoffset, Zscale);
    #endif
  }
  // use results to set compass.config(Xoffset,Yoffset,Zoffset,Xscale,Yscale,Zscale);
  /*
  // alternatively use scale from factory settings, see data sheet
  int8_t rom_adjX, rom_adjY, rom_adjZ;
  imu.getMagAdjustment(&rom_adjX, &rom_adjY, &rom_adjZ);
  float Xscale = (((int(rom_adjX)-128)*0.5)/128.0)+1;
  float Yscale = (((int(rom_adjY)-128)*0.5)/128.0)+1;
  float Zscale = (((int(rom_adjZ)-128)*0.5)/128.0)+1;
  */  
}

void esc_calibrate() {
    esc.update(motorPin_FL, throttleMax);
    esc.update(motorPin_FR, throttleMax);
    esc.update(motorPin_BR, throttleMax);
    esc.update(motorPin_BL, throttleMax);
    delay(3000);
    esc.update(motorPin_FL, 0);
    esc.update(motorPin_FR, 0);
    esc.update(motorPin_BR, 0);
    esc.update(motorPin_BL, 0);
}
