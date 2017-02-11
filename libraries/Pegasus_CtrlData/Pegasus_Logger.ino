#include "Wire.h"
#include "Pegasus_Log.h"

Pegasus_Log logger;

void setup() {
    Wire.begin();
    //Wire.setClock(400000L);
    Serial.begin(115200);
    while (!Serial);    
    logger.init();    
    randomSeed(micros());
}

unsigned long t;
float imu_x, imu_y, imu_z;
long pid_roll, pid_pitch, pid_yaw;
long mp_fl, mp_fr, mp_br, mp_bl;
float rx_roll, rx_pitch, rx_yaw;
unsigned short rx_throttle;
void loop() {
    t = millis();
    imu_x = 0.0001 * random(-1800000, 1800000);
    imu_y = 0.0001 * random(-1800000, 1800000);
    imu_z = 0.0001 * random(-1800000, 1800000);
    pid_roll = random(-65535, 65535);
    pid_pitch = random(-65535, 65535);
    pid_yaw = random(-65535, 65535);
    mp_fl = random(0, 65535);
    mp_fr = random(0, 65535);
    mp_br = random(0, 65535);
    mp_bl = random(0, 65535);
    rx_roll = 0.0001 * random(-450000, 450000);
    rx_pitch = 0.0001 * random(-450000, 450000);
    rx_yaw = 0.0001 * random(-450000, 450000);
    rx_throttle = random(0, 65535);
    
    Serial.printf("Append IMU %lu %0.2f %0.2f %0.2f\n", t, imu_x, imu_y, imu_z);
    logger.append_IMU( t, imu_x, imu_y, imu_z); logger.writeOnFull();
    
    Serial.printf("Append PID %lu %i %i %i\n", t, pid_roll, pid_pitch, pid_yaw);
    logger.append_PID( t, pid_roll, pid_pitch, pid_yaw); logger.writeOnFull();
    
    Serial.printf("Append MP %lu %u %u %u %u\n", t, mp_fl, mp_fr, mp_br, mp_bl);
    logger.append_MP( t, mp_fl, mp_fr, mp_br, mp_bl); logger.writeOnFull();
    
    Serial.printf("Append RX %lu %0.2f %0.2f %0.2f %u\n", t, rx_roll, rx_pitch, rx_yaw, rx_throttle);
    logger.append_RX( t, rx_roll, rx_pitch, rx_yaw, rx_throttle); logger.writeOnFull();
    //logger.writeOnFull();//logger.writeLog();//

    logger.readLog();
    delay(100);//while(true);//
}
