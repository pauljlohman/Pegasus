
#include "SPI.h"
#include "ZETA1.h"
#include "Pegasus_CtrlData.h"
#include "Stick.h"
#include "Wire.h"
#include "Pegasus_ConfigMem.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//#define DEBUG
#ifdef DEBUG
#define Serial_begin(x) Serial.begin(x);
#define print(x) Serial.print(x);
#else
#define Serial_begin(x);
#define print(x);
#endif

// general use buffers
byte buf[64];
short sbuf[3];
unsigned short usbuf[3];

ZETA1 radio(10, 8, 9); // CS, INT, SDN
Pegasus_ConfigMem mem;
Pegasus_CtrlData ctrl;

Stick stickYaw;
Stick stickPitch;
Stick stickRoll;
Stick stickThrottle;

byte rollPin = 8;
byte pitchPin = 7;
byte yawPin = 6;
byte throttlePin = 9;

Adafruit_SSD1306 display(6);

byte operatorPin = 17;
bool operatorButton_PS = false;
byte enterPin = 3;
bool enterButton_PS = false;
byte upPin = 4;
bool upButton_PS = false;
byte dnPin = 5;
bool dnButton_PS = false;
byte edMode = 0;
bool edModeEntered = false;

// Edit Modes
#define ED_MODE_YAW       0
#define ED_MODE_PITCH     1
#define ED_MODE_ROLL      2
#define ED_MODE_THROTTLE  3
#define ED_RANGE_YAW      4
#define ED_RANGE_PITCH    5
#define ED_RANGE_ROLL     6
#define ED_RANGE_THROTTLE 7
#define ED_PID_YAW_P      8
#define ED_PID_YAW_I      9
#define ED_PID_YAW_D      10
#define ED_PID_PITCH_P    11
#define ED_PID_PITCH_I    12
#define ED_PID_PITCH_D    13
#define ED_PID_ROLL_P     14
#define ED_PID_ROLL_I     15
#define ED_PID_ROLL_D     16
#define ED_MODE_COUNT 17
int32_t edMode_dirty = 0; // flip bit to set dirty
#define NAVMODE_YAW_BIT             0
#define NAVMODE_PITCH_BIT           1
#define NAVMODE_ROLL_BIT            2
#define NAVMODE_THROTTLE_BIT        3
#define NAVRANGE_YAW_REL_BIT        4
#define NAVRANGE_YAW_ABS_BIT        5
#define NAVRANGE_PITCH_REL_BIT      6
#define NAVRANGE_PITCH_ABS_BIT      7
#define NAVRANGE_ROLL_REL_BIT       8
#define NAVRANGE_ROLL_ABS_BIT       9
#define NAVRANGE_THROTTLE_REL_BIT   10
#define NAVRANGE_THROTTLE_ABS_BIT   11
#define PID_YAW_P_BIT               12
#define PID_YAW_I_BIT               13
#define PID_YAW_D_BIT               14
#define PID_PITCH_P_BIT             15
#define PID_PITCH_I_BIT             16
#define PID_PITCH_D_BIT             17
#define PID_ROLL_P_BIT              18
#define PID_ROLL_I_BIT              19
#define PID_ROLL_D_BIT              20

short pidGainStep = 1;
short navRangeStep = 1;
bool pendingUpdate_pidYaw = false;
bool pendingUpdate_pidPitch = false;
bool pendingUpdate_pidRoll = false;

void setup() {
  pinMode(operatorPin, INPUT);
  pinMode(enterPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);
  pinMode(dnPin, INPUT_PULLUP);

  Serial_begin(115200);
  //while(!Serial);
  print("Enter_Setup\n");
  
  // start I2C for EEPROM and display
  Wire.begin();

  // start SPI for TX
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); //1MHz seems to be stable

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("thinking");
  display.display();

  // EEPROM
  mem.init();
  //mem.format();

  // Config Radio
  print("TX_Setup...");
  radio.reset();
  while ( radio.SI4455_SUCCESS != radio.initialize()) {
    print("TX_Config_FAILED\n");
  }
  print("Complete\n");

  //Stick.config(byte pinX, float outerEdge, float deadZone, float range)
  stickYaw.config( yawPin, 0.0, 15.0, 2.0);
  stickPitch.config( pitchPin, 0.0, 15.0, 2.0);
  stickRoll.config( rollPin, 0.0, 15.0, 2.0);
  stickThrottle.config( throttlePin, 0.0, 15.0, 2.0);
  stickYaw.calibrate();
  stickPitch.calibrate();
  stickRoll.calibrate();
  stickThrottle.calibrate();
  print("stick config complete\n");

  loadConfigFromMem();
    
  display.clearDisplay();
  display.println("kk");
  display.display();

  print("Exit_Setup\n");  
}

const byte txPeriod = 21; // good @20kbs FSK
unsigned long txTimestamp = millis();
void loop() {
  pollInputs();
  if (millis() - txTimestamp > txPeriod) {
    txTimestamp = millis();
    sendPendingPacket();
  }
}

byte scrollPeriod = 100;
unsigned long lastScrollTime = millis();
void pollInputs(){
    // check kill switch
    bool operatorButton = digitalRead(operatorPin);
    bool operatorOnPress = !operatorButton_PS && operatorButton;
    bool operatorOnRelease = operatorButton_PS && !operatorButton;
    operatorButton_PS = operatorButton;
    if(operatorOnPress){ // on key press
        display.clearDisplay(); display.setCursor(0,0); display.println("Fly"); display.display();
        pendingUpdate_pidYaw = true;
        pendingUpdate_pidPitch = true;
        pendingUpdate_pidRoll = true;
        resetNav();
    }
    if(operatorOnRelease){ // on key release
        display.clearDisplay(); display.setCursor(0,0); display.println("Kill Power"); display.display();
    }

    // check editor buttons
    bool enterButton = !digitalRead(enterPin);
    bool enterOnRelease = enterButton_PS && !enterButton;
    enterButton_PS = enterButton;
    bool upButton = !digitalRead(upPin);
    bool dnButton = !digitalRead(dnPin);
    unsigned long elapsedScrollTime = millis() - lastScrollTime;
    
    if(edModeEntered){
        if(enterOnRelease){
            scrollPeriod = 200;
            edModeEntered = false;
            if(edMode_dirty){
                saveUpdatedParams();
            }
            displayEdModeData(false);
        }
        if(upButton){
            if(elapsedScrollTime > scrollPeriod){
                updateParams(true);
                displayEdModeData(true);
                lastScrollTime = millis();
            }
        }
        if(dnButton){
            if(elapsedScrollTime > scrollPeriod){
                updateParams(false);
                displayEdModeData(true);
                lastScrollTime = millis();
            }
        }
    }else{
        if(enterOnRelease){
            scrollPeriod = 50;
            edModeEntered = true;
            displayEdModeData(true);
        }
        if(upButton){
            if(elapsedScrollTime > scrollPeriod){
                edMode = (edMode+1 == ED_MODE_COUNT ? 0 : edMode+1);
                displayEdModeData(false);
                lastScrollTime = millis();
            }
        }
        if(dnButton){
            if(elapsedScrollTime > scrollPeriod){
                edMode = (edMode-1 == -1 ? ED_MODE_COUNT-1 : edMode-1);
                displayEdModeData(false);
                lastScrollTime = millis();
            }
        }
    }
}

void resetNav(){
  if (ctrl.navModeYaw == ctrl.FLY_REL){
    ctrl.yawf = 0.0;
  }
  if (ctrl.navModePitch == ctrl.FLY_REL){
    ctrl.pitchf = 0.0;
  }
  if (ctrl.navModeRoll == ctrl.FLY_REL){
    ctrl.rollf = 0.0;
  }
  if (ctrl.navModeThrottle == ctrl.FLY_REL){
    ctrl.throttlef = 0.0;
  }
}

void saveUpdatedParams(){
    if(bitRead(edMode_dirty, NAVMODE_YAW_BIT)){
        mem.write_NavMode_Yaw(ctrl.navModeYaw);
        bitClear(edMode_dirty, NAVMODE_YAW_BIT);
    }
    if(bitRead(edMode_dirty, NAVMODE_PITCH_BIT)){
        mem.write_NavMode_Pitch(ctrl.navModePitch);
        bitClear(edMode_dirty, NAVMODE_PITCH_BIT);
    }
    if(bitRead(edMode_dirty, NAVMODE_ROLL_BIT)){
        mem.write_NavMode_Roll(ctrl.navModeRoll);
        bitClear(edMode_dirty, NAVMODE_ROLL_BIT);
    }
    if(bitRead(edMode_dirty, NAVMODE_THROTTLE_BIT)){
        mem.write_NavMode_Throttle(ctrl.navModeThrottle);
        bitClear(edMode_dirty, NAVMODE_THROTTLE_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_YAW_REL_BIT)){
        mem.write_NavRange_Yaw_Rel(ctrl.navRangeYaw);
        bitClear(edMode_dirty, NAVRANGE_YAW_REL_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_YAW_ABS_BIT)){
        mem.write_NavRange_Yaw_Abs(ctrl.navRangeYaw);
        bitClear(edMode_dirty, NAVRANGE_YAW_ABS_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_PITCH_REL_BIT)){
        mem.write_NavRange_Pitch_Rel(ctrl.navRangePitch);
        bitClear(edMode_dirty, NAVRANGE_PITCH_REL_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_PITCH_ABS_BIT)){
        mem.write_NavRange_Pitch_Abs(ctrl.navRangePitch);
        bitClear(edMode_dirty, NAVRANGE_PITCH_ABS_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_ROLL_REL_BIT)){
        mem.write_NavRange_Roll_Rel(ctrl.navRangeRoll);
        bitClear(edMode_dirty, NAVRANGE_ROLL_REL_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_ROLL_ABS_BIT)){
        mem.write_NavRange_Roll_Abs(ctrl.navRangeRoll);
        bitClear(edMode_dirty, NAVRANGE_ROLL_ABS_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_THROTTLE_REL_BIT)){
        mem.write_NavRange_Throttle_Rel(ctrl.navRangeThrottle);
        bitClear(edMode_dirty, NAVRANGE_THROTTLE_REL_BIT);
    }
    if(bitRead(edMode_dirty, NAVRANGE_THROTTLE_ABS_BIT)){
        mem.write_NavRange_Throttle_Abs(ctrl.navRangeThrottle);
        bitClear(edMode_dirty, NAVRANGE_THROTTLE_ABS_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_P_BIT)){
        mem.write_PID_Yaw_P(ctrl.Kp_yaw);
        bitClear(edMode_dirty, PID_YAW_P_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_I_BIT)){
        mem.write_PID_Yaw_I(ctrl.Ki_yaw);
        bitClear(edMode_dirty, PID_YAW_I_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_D_BIT)){
        mem.write_PID_Yaw_D(ctrl.Kd_yaw);
        bitClear(edMode_dirty, PID_YAW_D_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_P_BIT)){
        mem.write_PID_Pitch_P(ctrl.Kp_pitch);
        bitClear(edMode_dirty, PID_PITCH_P_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_I_BIT)){
        mem.write_PID_Pitch_I(ctrl.Ki_pitch);
        bitClear(edMode_dirty, PID_PITCH_I_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_D_BIT)){
        mem.write_PID_Pitch_D(ctrl.Kd_pitch);
        bitClear(edMode_dirty, PID_PITCH_D_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_P_BIT)){
        mem.write_PID_Roll_P(ctrl.Kp_roll);
        bitClear(edMode_dirty, PID_ROLL_P_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_I_BIT)){
        mem.write_PID_Roll_I(ctrl.Ki_roll);
        bitClear(edMode_dirty, PID_ROLL_I_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_D_BIT)){
        mem.write_PID_Roll_D(ctrl.Kd_roll);
        bitClear(edMode_dirty, PID_ROLL_D_BIT);
    }
}

void updateParams(bool up){
    switch (edMode){
        // Nav Mode
        case ED_MODE_YAW:
            print("ED_MODE_YAW\n");
            ctrl.navModeYaw = up ? ctrl.FLY_ABS : ctrl.FLY_REL;
            bitSet(edMode_dirty, NAVMODE_YAW_BIT);
            switch(ctrl.navModeYaw){
                case ctrl.FLY_REL:
                    ctrl.navRangeYaw = mem.read_NavRange_Yaw_Rel();
                    break;
                case ctrl.FLY_ABS:
                    ctrl.navRangeYaw = mem.read_NavRange_Yaw_Abs();
                    break;
            }
            stickYaw.setRange( float(ctrl.navRangeYaw));
            break;
        case ED_MODE_PITCH:
            print("ED_MODE_PITCH\n");
            ctrl.navModePitch = up ? ctrl.FLY_ABS : ctrl.FLY_REL;
            bitSet(edMode_dirty, NAVMODE_PITCH_BIT);
            switch(ctrl.navModePitch){
                case ctrl.FLY_REL:
                    ctrl.navRangePitch = mem.read_NavRange_Pitch_Rel();
                    break;
                case ctrl.FLY_ABS:
                    ctrl.navRangePitch = mem.read_NavRange_Pitch_Abs();
                    break;
            }
            stickPitch.setRange( float(ctrl.navRangePitch));
            break;
        case ED_MODE_ROLL:
            print("ED_MODE_ROLL\n");
            ctrl.navModeRoll = up ? ctrl.FLY_ABS : ctrl.FLY_REL;
            bitSet(edMode_dirty, NAVMODE_ROLL_BIT);
            switch(ctrl.navModeRoll){
                case ctrl.FLY_REL:
                    ctrl.navRangeRoll = mem.read_NavRange_Roll_Rel();
                    break;
                case ctrl.FLY_ABS:
                    ctrl.navRangeRoll = mem.read_NavRange_Roll_Abs();
                    break;
            }
            stickRoll.setRange( float(ctrl.navRangeRoll));
            break;
        case ED_MODE_THROTTLE:
            print("ED_MODE_THROTTLE\n");
            ctrl.navModeThrottle = up ? ctrl.FLY_ABS : ctrl.FLY_REL;
            bitSet(edMode_dirty, NAVMODE_THROTTLE_BIT);
            switch(ctrl.navModeThrottle){
                case ctrl.FLY_REL:
                    ctrl.navRangeThrottle = mem.read_NavRange_Throttle_Rel();
                    break;
                case ctrl.FLY_ABS:
                    ctrl.navRangeThrottle = mem.read_NavRange_Throttle_Abs();
                    break;
            }
            stickThrottle.setRange( float(ctrl.navRangeThrottle));
            break;
            
        // Nav Range
        case ED_RANGE_YAW:
            print("ED_RANGE_YAW\n");
            ctrl.navRangeYaw += up ? navRangeStep : -navRangeStep;
            stickYaw.setRange( float(ctrl.navRangeYaw));
            switch(ctrl.navModeYaw){
                case ctrl.FLY_REL:
                    bitSet(edMode_dirty, NAVRANGE_YAW_REL_BIT);
                    break;
                case ctrl.FLY_ABS:
                    bitSet(edMode_dirty, NAVRANGE_YAW_ABS_BIT);
                    break;
            }
            break;
        case ED_RANGE_PITCH:
            print("ED_RANGE_PITCH\n");
            ctrl.navRangePitch += up ? navRangeStep : -navRangeStep;
            stickPitch.setRange( float(ctrl.navRangePitch));
            switch(ctrl.navModePitch){
                case ctrl.FLY_REL:
                    bitSet(edMode_dirty, NAVRANGE_PITCH_REL_BIT);
                    break;
                case ctrl.FLY_ABS:
                    bitSet(edMode_dirty, NAVRANGE_PITCH_ABS_BIT);
                    break;
            }
            break;
        case ED_RANGE_ROLL:
            print("ED_RANGE_ROLL\n");
            ctrl.navRangeRoll += up ? navRangeStep : -navRangeStep;
            stickRoll.setRange( float(ctrl.navRangeRoll));
            switch(ctrl.navModeRoll){
                case ctrl.FLY_REL:
                    bitSet(edMode_dirty, NAVRANGE_ROLL_REL_BIT);
                    break;
                case ctrl.FLY_ABS:
                    bitSet(edMode_dirty, NAVRANGE_ROLL_ABS_BIT);
                    break;
            }
            break;
        case ED_RANGE_THROTTLE:
            print("ED_RANGE_THROTTLE\n");
            ctrl.navRangeThrottle += up ? navRangeStep : -navRangeStep;
            stickThrottle.setRange( float(ctrl.navRangeThrottle));
            switch(ctrl.navModeThrottle){
                case ctrl.FLY_REL:
                    bitSet(edMode_dirty, NAVRANGE_THROTTLE_REL_BIT);
                    break;
                case ctrl.FLY_ABS:
                    bitSet(edMode_dirty, NAVRANGE_THROTTLE_ABS_BIT);
                    break;
            }
            break;
            
        // PID Yaw
        case ED_PID_YAW_P:
            print("ED_PID_YAW_P\n");
            ctrl.Kp_yaw += up ? pidGainStep : -pidGainStep;
            ctrl.Kp_yaw = max(min(32767, ctrl.Kp_yaw), 0);
            bitSet(edMode_dirty, PID_YAW_P_BIT);
            pendingUpdate_pidYaw = true;
            break;
        case ED_PID_YAW_I:
            print("ED_PID_YAW_I\n");
            ctrl.Ki_yaw += up ? pidGainStep : -pidGainStep;
            ctrl.Ki_yaw = max(min(32767, ctrl.Ki_yaw), 0);
            bitSet(edMode_dirty, PID_YAW_I_BIT);
            pendingUpdate_pidYaw = true;
            break;
        case ED_PID_YAW_D:
            print("ED_PID_YAW_D\n");
            ctrl.Kd_yaw += up ? pidGainStep : -pidGainStep;
            ctrl.Kd_yaw = max(min(32767, ctrl.Kd_yaw), 0);
            bitSet(edMode_dirty, PID_YAW_D_BIT);
            pendingUpdate_pidYaw = true;
            break;

        // PID Pitch
        case ED_PID_PITCH_P:
            print("ED_PID_PITCH_P\n");
            ctrl.Kp_pitch += up ? pidGainStep : -pidGainStep;
            ctrl.Kp_pitch = max(min(32767, ctrl.Kp_pitch), 0);
            bitSet(edMode_dirty, PID_PITCH_P_BIT);
            pendingUpdate_pidPitch = true;
            break;
        case ED_PID_PITCH_I:
            print("ED_PID_PITCH_I\n");
            ctrl.Ki_pitch += up ? pidGainStep : -pidGainStep;
            ctrl.Ki_pitch = max(min(32767, ctrl.Ki_pitch), 0);
            bitSet(edMode_dirty, PID_PITCH_I_BIT);
            pendingUpdate_pidPitch = true;
            break;
        case ED_PID_PITCH_D:
            print("ED_PID_PITCH_D\n");
            ctrl.Kd_pitch += up ? pidGainStep : -pidGainStep;
            ctrl.Kd_pitch = max(min(32767, ctrl.Kd_pitch), 0);
            bitSet(edMode_dirty, PID_PITCH_D_BIT);
            pendingUpdate_pidPitch = true;
            break;

        // PID Roll
        case ED_PID_ROLL_P:
            print("ED_PID_ROLL_P\n");
            ctrl.Kp_roll += up ? pidGainStep : -pidGainStep;
            ctrl.Kp_roll = max(ctrl.Kp_roll, 0);
            bitSet(edMode_dirty, PID_ROLL_P_BIT);
            pendingUpdate_pidRoll = true;
            break;
        case ED_PID_ROLL_I:
            print("ED_PID_ROLL_I\n");
            ctrl.Ki_roll += up ? pidGainStep : -pidGainStep;
            ctrl.Ki_roll = max(ctrl.Ki_roll, 0);
            bitSet(edMode_dirty, PID_ROLL_I_BIT);
            pendingUpdate_pidRoll = true;
            break;
        case ED_PID_ROLL_D:
            print("ED_PID_ROLL_D\n");
            ctrl.Kd_roll += up ? pidGainStep : -pidGainStep;
            ctrl.Kd_roll = max(ctrl.Kd_roll, 0);
            bitSet(edMode_dirty, PID_ROLL_D_BIT);
            pendingUpdate_pidRoll = true;
            break;
    }
}

void sendPendingPacket(){
    if(pendingUpdate_pidYaw){
        pendingUpdate_pidYaw = false;
        sendPacket_PID_yaw();
        return;
    }
    if(pendingUpdate_pidPitch){
        pendingUpdate_pidPitch = false;
        sendPacket_PID_pitch();
        return;
    }
    if(pendingUpdate_pidRoll){
        pendingUpdate_pidRoll = false;
        sendPacket_PID_roll();
        return;
    }
    if(operatorButton_PS){
      sendPacket_Nav();
    } else {
      sendPacket_Kill();
    }
}

void sendPacket_Nav() {
  ctrl.control_code = ctrl.NAV_CODE;




  stickYaw.update();
  stickPitch.update();
  stickRoll.update();
  stickThrottle.update();
  //Serial.printf("left x:%f y:%f\t right x:%f y:%f\n", stickYaw.v, stickPitch.v, stickRoll.v, stickThrottle.v);
  switch (ctrl.navModeYaw){
    case ctrl.FLY_ABS: 
        ctrl.yawf = stickYaw.v * -1.0; break;
    case ctrl.FLY_REL:
        ctrl.yawf = wrapAngle(ctrl.yawf + (stickYaw.v * -1) ); break;
  }
  ctrl.buf[0] = int(ctrl.yawf * 100.0);

  switch (ctrl.navModePitch){
    case ctrl.FLY_ABS: 
        ctrl.pitchf = stickPitch.v * -1; break;
    case ctrl.FLY_REL:
        ctrl.pitchf = wrapAngle(ctrl.pitchf + (stickPitch.v * -1)); break;
  }
  ctrl.buf[1] = int(ctrl.pitchf * 100.0);
  
  switch (ctrl.navModeRoll){
    case ctrl.FLY_ABS:
        ctrl.rollf = stickRoll.v * -1; break;
    case ctrl.FLY_REL:
        ctrl.rollf = wrapAngle(ctrl.rollf + (stickRoll.v * -1)); break;
  }
  ctrl.buf[2] = int(ctrl.rollf * 100.0);
  
  switch (ctrl.navModeThrottle){
    case ctrl.FLY_ABS: 
        ctrl.throttlef = max(stickThrottle.v * 2 * -1, 0.0); break;
    case ctrl.FLY_REL:
        ctrl.throttlef += stickThrottle.v * -1; 
        ctrl.throttlef = max(ctrl.throttlef, 0.0);
        ctrl.throttlef = min(ctrl.throttlef, 65535.0);
        break;
  }
  ctrl.buf[3] = int(ctrl.throttlef);
  ctrl.pack(buf);
  radio.startTX(buf);
  
  //Serial.printf("NAVp %i %i %i %i %i\n", ctrl.buf[0], ctrl.buf[1], ctrl.buf[2], ctrl.buf[3], millis());
  print("NAV ");
  print(ctrl.yawf); print(" ");
  print(ctrl.pitchf); print(" ");
  print(ctrl.rollf); print(" ");
  print(ctrl.throttlef); print(" ");
  print(millis()); print("\n");
}
void sendPacket_PID_yaw() {
  print("PID_yaw ");
  sendPacket_PID(ctrl.Kp_yaw, ctrl.Ki_yaw, ctrl.Kd_yaw, ctrl.PID_YAW_CODE);
}
void sendPacket_PID_pitch() {
  print("PID_pitch ");
  sendPacket_PID(ctrl.Kp_pitch, ctrl.Ki_pitch, ctrl.Kd_pitch, ctrl.PID_PITCH_CODE);
}
void sendPacket_PID_roll() {
  print("PID_roll ");
  sendPacket_PID(ctrl.Kp_roll, ctrl.Ki_roll, ctrl.Kd_roll, ctrl.PID_ROLL_CODE);
}
void sendPacket_PID(short p, short i, short d, byte code) {
  ctrl.control_code = code;
  ctrl.buf[0] = p;
  ctrl.buf[1] = i;
  ctrl.buf[2] = d;
  ctrl.pack(buf);
  radio.startTX(buf);
  
  print(p);   print(" ");
  print(i); print(" ");
  print(d);  print("\n");  
}
void sendPacket_Kill() {
  ctrl.control_code = ctrl.KILL_CODE;
  ctrl.pack(buf);
  //print("KILL_CODE \n");
  radio.startTX(buf);
}

void loadConfigFromMem(){
    print("loadConfigFromMem\n");
    // Nav Mode
    ctrl.navModeYaw = mem.read_NavMode_Yaw();
    ctrl.navModePitch = mem.read_NavMode_Pitch();
    ctrl.navModeRoll = mem.read_NavMode_Roll();
    ctrl.navModeThrottle = mem.read_NavMode_Throttle();
    print("NavMode yaw: "); print(ctrl.navModeYaw); 
    print(" pitch: "); print(ctrl.navModePitch); 
    print(" roll: "); print(ctrl.navModeRoll); 
    print(" throttle: "); print(ctrl.navModeThrottle); print("\n");

    // Nav Range (byte) & PID settings per Nav Mode (unsigned short)
    switch (ctrl.navModeYaw) {
        case ctrl.FLY_ABS:
            ctrl.navRangeYaw = mem.read_NavRange_Yaw_Abs(); break;
        case ctrl.FLY_REL:
            ctrl.navRangeYaw = mem.read_NavRange_Yaw_Rel(); break;
    }
    stickYaw.setRange( float(ctrl.navRangeYaw));
    
    switch (ctrl.navModePitch) {
        case ctrl.FLY_ABS:
            ctrl.navRangePitch = mem.read_NavRange_Pitch_Abs(); break;
        case ctrl.FLY_REL:
            ctrl.navRangePitch = mem.read_NavRange_Pitch_Rel(); break;
    }
    stickPitch.setRange( float(ctrl.navRangePitch));
    
    switch (ctrl.navModeRoll) {
        case ctrl.FLY_ABS:
            ctrl.navRangeRoll = mem.read_NavRange_Roll_Abs(); break;
        case ctrl.FLY_REL:
            ctrl.navRangeRoll = mem.read_NavRange_Roll_Rel(); break;
    }
    stickRoll.setRange( float(ctrl.navRangeRoll));
    
    switch (ctrl.navModeThrottle) {
        case ctrl.FLY_ABS:
            ctrl.navRangeThrottle = mem.read_NavRange_Throttle_Abs(); break;
        case ctrl.FLY_REL:
            ctrl.navRangeThrottle = mem.read_NavRange_Throttle_Rel(); break;
    }
    stickThrottle.setRange( float(ctrl.navRangeThrottle));
    
    print("NavRange yaw: "); print(ctrl.navRangeYaw); 
    print(" pitch: "); print(ctrl.navRangePitch); 
    print(" roll: "); print(ctrl.navRangeRoll); 
    print(" throttle: "); print(ctrl.navRangeThrottle); print("\n");    
    
    ctrl.Kp_yaw   = mem.read_PID_Yaw_P();
    ctrl.Ki_yaw   = mem.read_PID_Yaw_I();
    ctrl.Kd_yaw   = mem.read_PID_Yaw_D();
    ctrl.Kp_pitch = mem.read_PID_Pitch_P();
    ctrl.Ki_pitch = mem.read_PID_Pitch_I();
    ctrl.Kd_pitch = mem.read_PID_Pitch_D();
    ctrl.Kp_roll  = mem.read_PID_Roll_P();
    ctrl.Ki_roll  = mem.read_PID_Roll_I();
    ctrl.Kd_roll  = mem.read_PID_Roll_D();
    print("PID Yaw   - P:"); print(ctrl.Kp_yaw);   print(" I:"); print(ctrl.Ki_yaw);   print(" D:"); print(ctrl.Kd_yaw);   print("\n");
    print("PID Pitch - P:"); print(ctrl.Kp_pitch); print(" I:"); print(ctrl.Ki_pitch); print(" D:"); print(ctrl.Kd_pitch); print("\n");
    print("PID Roll  - P:"); print(ctrl.Kp_roll);  print(" I:"); print(ctrl.Ki_roll);  print(" D:"); print(ctrl.Kd_roll);  print("\n");

};

void displayEdModeData(bool showData){
    display.clearDisplay();
    display.setCursor(0,0);
    switch (edMode){
        // Nav Mode
        case ED_MODE_YAW:
            display.println("Flight Mode Yaw");
            if(showData){ displayNavMode(ctrl.navModeYaw); }
            break;
        case ED_MODE_PITCH:
            display.println("Flight Mode Pitch");
            if(showData){ displayNavMode(ctrl.navModePitch); }
            break;
        case ED_MODE_ROLL:
            display.println("Flight Mode Roll");
            if(showData){ displayNavMode(ctrl.navModeRoll); }
            break;
        case ED_MODE_THROTTLE:
            display.println("Flight Mode Throttle");
            if(showData){ displayNavMode(ctrl.navModeThrottle); }
            break;
            
        // Nav Rotation Range
        case ED_RANGE_YAW:
            display.println("RC Range Yaw");
            if(showData){
                display.println(ctrl.navRangeYaw);
                displayNavMode(ctrl.navModeYaw);
            }
            break;
        case ED_RANGE_PITCH:
            display.println("RC Range Pitch");
            if(showData){
                display.println(ctrl.navRangePitch);
                displayNavMode(ctrl.navModePitch);
            }
            break;
        case ED_RANGE_ROLL:
            display.println("RC Range Roll");
            if(showData){
                display.println(ctrl.navRangeRoll);
                displayNavMode(ctrl.navModeRoll);
            }
            break;
        case ED_RANGE_THROTTLE:
            display.println("RC Range Throttle");
            if(showData){
                display.println(ctrl.navRangeThrottle);
                displayNavMode(ctrl.navModeThrottle);
            }
            break;
            
        // PID Yaw
        case ED_PID_YAW_P:
            display.println("PID Yaw P");
            if(showData){display.println(ctrl.Kp_yaw); }
            break;
        case ED_PID_YAW_I:
            display.println("PID Yaw I");
            if(showData){display.println(ctrl.Ki_yaw); }
            break;
        case ED_PID_YAW_D:
            display.println("PID Yaw D");
            if(showData){display.println(ctrl.Kd_yaw); }
            break;

        // PID Pitch
        case ED_PID_PITCH_P:
            display.println("PID Pitch P");
            if(showData){display.println(ctrl.Kp_pitch); }
            break;
        case ED_PID_PITCH_I:
            display.println("PID Pitch I");
            if(showData){display.println(ctrl.Ki_pitch); }
            break;
        case ED_PID_PITCH_D:
            display.println("PID Pitch D");
            if(showData){display.println(ctrl.Kd_pitch); }
            break;

        // PID Roll
        case ED_PID_ROLL_P:
            display.println("PID Roll P");
            if(showData){display.println(ctrl.Kp_roll); }
            break;
        case ED_PID_ROLL_I:
            display.println("PID Roll I");
            if(showData){display.println(ctrl.Ki_roll); }
            break;
        case ED_PID_ROLL_D:
            display.println("PID Roll D");
            if(showData){display.println(ctrl.Kd_roll); }
            break;
    }
    display.display();
}

void displayNavMode(byte mode){
    switch (mode){
        case ctrl.FLY_ABS:
            display.println("Absolute");
            break;
        case ctrl.FLY_REL:
            display.println("Relative");
            break;
    }
}

float wrapAngle(float v){
    if(v > 180.0){
        return v - 360.0;
    }
    if(v < -180.0){
        return v + 360.0;
    }
    return v;
};
