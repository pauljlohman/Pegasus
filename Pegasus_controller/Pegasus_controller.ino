
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

Stick stickLeft;
Stick stickRight;

byte rollPin = 8;
byte pitchPin = 7;
byte yawPin = 6;
byte throttlePin = 9;

Adafruit_SSD1306 display(6);

byte saftyPin = 17;
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
#define ED_ROTMODE_YAW     0
#define ED_ROTMODE_PITCH   1
#define ED_ROTMODE_ROLL    2
#define ED_ROTRANGE_YAW    3
#define ED_ROTRANGE_PITCH  4
#define ED_ROTRANGE_ROLL   5
#define ED_PID_YAW_P       6
#define ED_PID_YAW_I       7
#define ED_PID_YAW_D       8
#define ED_PID_PITCH_P     9
#define ED_PID_PITCH_I     10
#define ED_PID_PITCH_D     11
#define ED_PID_ROLL_P      12
#define ED_PID_ROLL_I      13
#define ED_PID_ROLL_D      14
#define ED_MODE_COUNT 15
int32_t edMode_dirty = 0; // flip bit to set dirty
#define ROTMODE_YAW_BIT          0
#define ROTMODE_PITCH_BIT        1
#define ROTMODE_ROLL_BIT         2
#define ROTRANGE_YAW_ACRO_BIT    3
#define ROTRANGE_YAW_STAB_BIT    4
#define ROTRANGE_PITCH_ACRO_BIT  5
#define ROTRANGE_PITCH_STAB_BIT  6
#define ROTRANGE_ROLL_ACRO_BIT   7
#define ROTRANGE_ROLL_STAB_BIT   8
#define PID_YAW_P_ACRO_BIT       9
#define PID_YAW_P_STAB_BIT       10
#define PID_YAW_I_ACRO_BIT       11
#define PID_YAW_I_STAB_BIT       12
#define PID_YAW_D_ACRO_BIT       13
#define PID_YAW_D_STAB_BIT       14
#define PID_PITCH_P_ACRO_BIT     15
#define PID_PITCH_P_STAB_BIT     16
#define PID_PITCH_I_ACRO_BIT     17
#define PID_PITCH_I_STAB_BIT     18
#define PID_PITCH_D_ACRO_BIT     19
#define PID_PITCH_D_STAB_BIT     20
#define PID_ROLL_P_ACRO_BIT      21
#define PID_ROLL_P_STAB_BIT      22
#define PID_ROLL_I_ACRO_BIT      23
#define PID_ROLL_I_STAB_BIT      24
#define PID_ROLL_D_ACRO_BIT      25
#define PID_ROLL_D_STAB_BIT      26

float pidGainStep = 1.0 / ctrl.pidScale;
short navRotRangeStep = 2;
bool pendingUpdate_navRotMode = false;
bool pendingUpdate_navRotRange = false;
bool pendingUpdate_pidYaw = false;
bool pendingUpdate_pidPitch = false;
bool pendingUpdate_pidRoll = false;

void setup() {
  pinMode(saftyPin, INPUT);
  pinMode(enterPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);
  pinMode(dnPin, INPUT_PULLUP);

  // start I2C for EEPROM and display
  Wire.begin();

  // start SPI for TX
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); //1MHz seems to be stable

  Serial_begin(115200);//while(!Serial);
  print("Enter_Setup\n");

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
  loadConfigFromMem();

  // Config Radio
  print("TX_Setup...");
  radio.reset();
  while ( radio.SI4455_SUCCESS != radio.initialize()) {
    print("TX_Config_FAILED\n");
  }
  print("Complete\n");

  //Stick.config(byte _pinX, byte _pinY, float _outerEdge, float _deadZone, float _dampen)
  stickLeft.config( yawPin, throttlePin, 0.0, 15.0, 0.5);
  stickRight.config( rollPin, pitchPin,  0.0, 15.0, 0.5);
  stickLeft.calibrate();
  stickRight.calibrate();
  print("stick config complete\n");

  display.clearDisplay();
  display.println("kk");
  display.display();
  print("Exit_Setup\n");
}

const byte txPeriod = 21; // good @20kbs FSK
unsigned long txTimestamp = millis();
void loop() {
    
  if (millis() - txTimestamp > txPeriod) {
    txTimestamp = millis();
    sendPendingPacket();
  }
  
  pollInputs();
  
}

byte scrollPeriod = 100;
unsigned long lastScrollTime = millis();
void pollInputs(){
    // check kill switch
    bool operatorButton = digitalRead(saftyPin);
    bool operatorOnPress = !operatorButton_PS && operatorButton;
    bool operatorOnRelease = operatorButton_PS && !operatorButton;
    operatorButton_PS = operatorButton;
    if(operatorOnPress){ // on key press
        display.clearDisplay(); display.setCursor(0,0); display.println("Fly"); display.display();
        pendingUpdate_navRotMode = true;
        pendingUpdate_navRotRange = true;
        pendingUpdate_pidYaw = true;
        pendingUpdate_pidPitch = true;
        pendingUpdate_pidRoll = true;
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
            scrollPeriod = 100;
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
            scrollPeriod = 20;
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

void saveUpdatedParams(){
    if(bitRead(edMode_dirty, ROTMODE_YAW_BIT)){
        mem.write_RotMode_Yaw(ctrl.navRotModeYaw);
        bitClear(edMode_dirty, ROTMODE_YAW_BIT);
    }
    if(bitRead(edMode_dirty, ROTMODE_PITCH_BIT)){
        mem.write_RotMode_Pitch(ctrl.navRotModePitch);
        bitClear(edMode_dirty, ROTMODE_PITCH_BIT);
    }
    if(bitRead(edMode_dirty, ROTMODE_ROLL_BIT)){
        mem.write_RotMode_Roll(ctrl.navRotModeRoll);
        bitClear(edMode_dirty, ROTMODE_ROLL_BIT);
    }
    if(bitRead(edMode_dirty, ROTRANGE_YAW_ACRO_BIT)){
        mem.write_RotRange_Yaw_Acro(ctrl.navRotRangeYaw);
        bitClear(edMode_dirty, ROTRANGE_YAW_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, ROTRANGE_YAW_STAB_BIT)){
        mem.write_RotRange_Yaw_Stab(ctrl.navRotRangeYaw);
        bitClear(edMode_dirty, ROTRANGE_YAW_STAB_BIT);
    }
    if(bitRead(edMode_dirty, ROTRANGE_PITCH_ACRO_BIT)){
        mem.write_RotRange_Pitch_Acro(ctrl.navRotRangePitch);
        bitClear(edMode_dirty, ROTRANGE_PITCH_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, ROTRANGE_PITCH_STAB_BIT)){
        mem.write_RotRange_Pitch_Stab(ctrl.navRotRangePitch);
        bitClear(edMode_dirty, ROTRANGE_PITCH_STAB_BIT);
    }
    if(bitRead(edMode_dirty, ROTRANGE_ROLL_ACRO_BIT)){
        mem.write_RotRange_Roll_Acro(ctrl.navRotRangeRoll);
        bitClear(edMode_dirty, ROTRANGE_ROLL_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, ROTRANGE_ROLL_STAB_BIT)){
        mem.write_RotRange_Roll_Stab(ctrl.navRotRangeRoll);
        bitClear(edMode_dirty, ROTRANGE_ROLL_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_P_ACRO_BIT)){
        mem.write_PID_Yaw_P_Acro(int(ctrl.Kp_yaw*ctrl.pidScale));
        bitClear(edMode_dirty, PID_YAW_P_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_P_STAB_BIT)){
        mem.write_PID_Yaw_P_Stab(int(ctrl.Kp_yaw*ctrl.pidScale));
        bitClear(edMode_dirty, PID_YAW_P_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_I_ACRO_BIT)){
        mem.write_PID_Yaw_I_Acro(int(ctrl.Ki_yaw*ctrl.pidScale));
        bitClear(edMode_dirty, PID_YAW_I_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_I_STAB_BIT)){
        mem.write_PID_Yaw_I_Stab(int(ctrl.Ki_yaw*ctrl.pidScale));
        bitClear(edMode_dirty, PID_YAW_I_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_D_ACRO_BIT)){
        mem.write_PID_Yaw_D_Acro(int(ctrl.Kd_yaw*ctrl.pidScale));
        bitClear(edMode_dirty, PID_YAW_D_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_YAW_D_STAB_BIT)){
        mem.write_PID_Yaw_D_Stab(int(ctrl.Kd_yaw*ctrl.pidScale));
        bitClear(edMode_dirty, PID_YAW_D_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_P_ACRO_BIT)){
        mem.write_PID_Pitch_P_Acro(int(ctrl.Kp_pitch*ctrl.pidScale));
        bitClear(edMode_dirty, PID_PITCH_P_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_P_STAB_BIT)){
        mem.write_PID_Pitch_P_Stab(int(ctrl.Kp_pitch*ctrl.pidScale));
        bitClear(edMode_dirty, PID_PITCH_P_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_I_ACRO_BIT)){
        mem.write_PID_Pitch_I_Acro(int(ctrl.Ki_pitch*ctrl.pidScale));
        bitClear(edMode_dirty, PID_PITCH_I_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_I_STAB_BIT)){
        mem.write_PID_Pitch_I_Stab(int(ctrl.Ki_pitch*ctrl.pidScale));
        bitClear(edMode_dirty, PID_PITCH_I_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_D_ACRO_BIT)){
        mem.write_PID_Pitch_D_Acro(int(ctrl.Kd_pitch*ctrl.pidScale));
        bitClear(edMode_dirty, PID_PITCH_D_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_PITCH_D_STAB_BIT)){
        mem.write_PID_Pitch_D_Stab(int(ctrl.Kd_pitch*ctrl.pidScale));
        bitClear(edMode_dirty, PID_PITCH_D_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_P_ACRO_BIT)){
        mem.write_PID_Roll_P_Acro(int(ctrl.Kp_roll*ctrl.pidScale));
        bitClear(edMode_dirty, PID_ROLL_P_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_P_STAB_BIT)){
        mem.write_PID_Roll_P_Stab(int(ctrl.Kp_roll*ctrl.pidScale));
        bitClear(edMode_dirty, PID_ROLL_P_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_I_ACRO_BIT)){
        mem.write_PID_Roll_I_Acro(int(ctrl.Ki_roll*ctrl.pidScale));
        bitClear(edMode_dirty, PID_ROLL_I_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_I_STAB_BIT)){
        mem.write_PID_Roll_I_Stab(int(ctrl.Ki_roll*ctrl.pidScale));
        bitClear(edMode_dirty, PID_ROLL_I_STAB_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_D_ACRO_BIT)){
        mem.write_PID_Roll_D_Acro(int(ctrl.Kd_roll*ctrl.pidScale));
        bitClear(edMode_dirty, PID_ROLL_D_ACRO_BIT);
    }
    if(bitRead(edMode_dirty, PID_ROLL_D_STAB_BIT)){
        mem.write_PID_Roll_D_Stab(int(ctrl.Kd_roll*ctrl.pidScale));
        bitClear(edMode_dirty, PID_ROLL_D_STAB_BIT);
    }
}

void updateParams(bool up){
    switch (edMode){
        // Nav Mode
        case ED_ROTMODE_YAW:
            print("ED_ROTMODE_YAW\n");
            ctrl.navRotModeYaw = up ? ctrl.FLY_STAB : ctrl.FLY_ACRO;
            bitSet(edMode_dirty, ROTMODE_YAW_BIT);
            switch(ctrl.navRotModeYaw){
                case ctrl.FLY_ACRO:
                    ctrl.navRotRangeYaw = mem.read_RotRange_Yaw_Acro();
                    ctrl.Kp_yaw = mem.read_PID_Yaw_P_Acro() / ctrl.pidScale;
                    ctrl.Ki_yaw = mem.read_PID_Yaw_I_Acro() / ctrl.pidScale;
                    ctrl.Kd_yaw = mem.read_PID_Yaw_D_Acro() / ctrl.pidScale;
                    break;
                case ctrl.FLY_STAB:
                    ctrl.navRotRangeYaw = mem.read_RotRange_Yaw_Stab();
                    ctrl.Kp_yaw = mem.read_PID_Yaw_P_Stab() / ctrl.pidScale;
                    ctrl.Ki_yaw = mem.read_PID_Yaw_I_Stab() / ctrl.pidScale;
                    ctrl.Kd_yaw = mem.read_PID_Yaw_D_Stab() / ctrl.pidScale;
                    break;
            }
            pendingUpdate_navRotMode = true;
            pendingUpdate_navRotRange = true;
            pendingUpdate_pidYaw = true;
            break;
        case ED_ROTMODE_PITCH:
            print("ED_ROTMODE_PITCH\n");
            ctrl.navRotModePitch = up ? ctrl.FLY_STAB : ctrl.FLY_ACRO;
            bitSet(edMode_dirty, ROTMODE_PITCH_BIT);
            switch(ctrl.navRotModePitch){
                case ctrl.FLY_ACRO:
                    ctrl.navRotRangePitch = mem.read_RotRange_Pitch_Acro();
                    ctrl.Kp_pitch = mem.read_PID_Pitch_P_Acro() / ctrl.pidScale;
                    ctrl.Ki_pitch = mem.read_PID_Pitch_I_Acro() / ctrl.pidScale;
                    ctrl.Kd_pitch = mem.read_PID_Pitch_D_Acro() / ctrl.pidScale;
                    break;
                case ctrl.FLY_STAB:
                    ctrl.navRotRangePitch = mem.read_RotRange_Pitch_Stab();
                    ctrl.Kp_pitch = mem.read_PID_Pitch_P_Stab() / ctrl.pidScale;
                    ctrl.Ki_pitch = mem.read_PID_Pitch_I_Stab() / ctrl.pidScale;
                    ctrl.Kd_pitch = mem.read_PID_Pitch_D_Stab() / ctrl.pidScale;
                    break;
            }
            pendingUpdate_navRotMode = true;
            pendingUpdate_navRotRange = true;
            pendingUpdate_pidPitch = true;
            break;
        case ED_ROTMODE_ROLL:
            print("ED_ROTMODE_ROLL\n");
            ctrl.navRotModeRoll = up ? ctrl.FLY_STAB : ctrl.FLY_ACRO;
            bitSet(edMode_dirty, ROTMODE_ROLL_BIT);
            switch(ctrl.navRotModeRoll){
                case ctrl.FLY_ACRO:
                    ctrl.navRotRangeRoll = mem.read_RotRange_Roll_Acro();
                    ctrl.Kp_roll = mem.read_PID_Roll_P_Acro() / ctrl.pidScale;
                    ctrl.Ki_roll = mem.read_PID_Roll_I_Acro() / ctrl.pidScale;
                    ctrl.Kd_roll = mem.read_PID_Roll_D_Acro() / ctrl.pidScale;
                    break;
                case ctrl.FLY_STAB:
                    ctrl.navRotRangeRoll = mem.read_RotRange_Roll_Stab();
                    ctrl.Kp_roll = mem.read_PID_Roll_P_Stab() / ctrl.pidScale;
                    ctrl.Ki_roll = mem.read_PID_Roll_I_Stab() / ctrl.pidScale;
                    ctrl.Kd_roll = mem.read_PID_Roll_D_Stab() / ctrl.pidScale;
                    break;
            }
            pendingUpdate_navRotMode = true;
            pendingUpdate_navRotRange = true;
            pendingUpdate_pidRoll = true;
            break;

        // Nav Rotation Range
        case ED_ROTRANGE_YAW:
            print("ED_ROTRANGE_YAW\n");
            ctrl.navRotRangeYaw += up ? navRotRangeStep : -navRotRangeStep;
            switch(ctrl.navRotModeYaw){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, ROTRANGE_YAW_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, ROTRANGE_YAW_STAB_BIT);
                    break;
            }
            pendingUpdate_navRotRange = true;
            break;
        case ED_ROTRANGE_PITCH:
            print("ED_ROTRANGE_PITCH\n");
            ctrl.navRotRangePitch += up ? navRotRangeStep : -navRotRangeStep;
            switch(ctrl.navRotModePitch){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, ROTRANGE_PITCH_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, ROTRANGE_PITCH_STAB_BIT);
                    break;
            }
            pendingUpdate_navRotRange = true;
            break;
        case ED_ROTRANGE_ROLL:
            print("ED_ROTRANGE_ROLL\n");
            ctrl.navRotRangeRoll += up ? navRotRangeStep : -navRotRangeStep;
            switch(ctrl.navRotModeRoll){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, ROTRANGE_ROLL_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, ROTRANGE_ROLL_STAB_BIT);
                    break;
            }
            pendingUpdate_navRotRange = true;
            break;

        // PID Yaw
        case ED_PID_YAW_P:
            print("ED_PID_YAW_P\n");
            ctrl.Kp_yaw += up ? pidGainStep : -pidGainStep;
            ctrl.Kp_yaw = max(min(32767.0/ctrl.pidScale, ctrl.Kp_yaw), 0.0);
            switch(ctrl.navRotModeYaw){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_YAW_P_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_YAW_P_STAB_BIT);
                    break;
            }
            pendingUpdate_pidYaw = true;
            break;
        case ED_PID_YAW_I:
            print("ED_PID_YAW_I\n");
            ctrl.Ki_yaw += up ? pidGainStep : -pidGainStep;
            ctrl.Ki_yaw = max(min(32767.0/ctrl.pidScale, ctrl.Ki_yaw), 0.0);
            switch(ctrl.navRotModeYaw){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_YAW_I_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_YAW_I_STAB_BIT);
                    break;
            }
            pendingUpdate_pidYaw = true;
            break;
        case ED_PID_YAW_D:
            print("ED_PID_YAW_D\n");
            ctrl.Kd_yaw += up ? pidGainStep : -pidGainStep;
            ctrl.Kd_yaw = max(min(32767.0/ctrl.pidScale, ctrl.Kd_yaw), 0.0);
            switch(ctrl.navRotModeYaw){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_YAW_D_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_YAW_D_STAB_BIT);
                    break;
            }
            pendingUpdate_pidYaw = true;
            break;

        // PID Pitch
        case ED_PID_PITCH_P:
            print("ED_PID_PITCH_P\n");
            ctrl.Kp_pitch += up ? pidGainStep : -pidGainStep;
            ctrl.Kp_pitch = max(min(32767.0/ctrl.pidScale, ctrl.Kp_pitch), 0.0);
            switch(ctrl.navRotModePitch){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_PITCH_P_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_PITCH_P_STAB_BIT);
                    break;
            }
            pendingUpdate_pidPitch = true;
            break;
        case ED_PID_PITCH_I:
            print("ED_PID_PITCH_I\n");
            ctrl.Ki_pitch += up ? pidGainStep : -pidGainStep;
            ctrl.Ki_pitch = max(min(32767.0/ctrl.pidScale, ctrl.Ki_pitch), 0.0);
            switch(ctrl.navRotModePitch){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_PITCH_I_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_PITCH_I_STAB_BIT);
                    break;
            }
            pendingUpdate_pidPitch = true;
            break;
        case ED_PID_PITCH_D:
            print("ED_PID_PITCH_D\n");
            ctrl.Kd_pitch += up ? pidGainStep : -pidGainStep;
            ctrl.Kd_pitch = max(min(32767.0/ctrl.pidScale, ctrl.Kd_pitch), 0.0);
            switch(ctrl.navRotModePitch){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_PITCH_D_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_PITCH_D_STAB_BIT);
                    break;
            }
            pendingUpdate_pidPitch = true;
            break;

        // PID Roll
        case ED_PID_ROLL_P:
            print("ED_PID_ROLL_P\n");
            ctrl.Kp_roll += up ? pidGainStep : -pidGainStep;
            ctrl.Kp_roll = max(ctrl.Kp_roll, 0.0);
            switch(ctrl.navRotModeRoll){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_ROLL_P_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_ROLL_P_STAB_BIT);
                    break;
            }
            pendingUpdate_pidRoll = true;
            break;
        case ED_PID_ROLL_I:
            print("ED_PID_ROLL_I\n");
            ctrl.Ki_roll += up ? pidGainStep : -pidGainStep;
            ctrl.Ki_roll = max(ctrl.Ki_roll, 0.0);
            switch(ctrl.navRotModeRoll){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_ROLL_I_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_ROLL_I_STAB_BIT);
                    break;
            }
            pendingUpdate_pidRoll = true;
            break;
        case ED_PID_ROLL_D:
            print("ED_PID_ROLL_D\n");
            ctrl.Kd_roll += up ? pidGainStep : -pidGainStep;
            ctrl.Kd_roll = max(ctrl.Kd_roll, 0.0);
            switch(ctrl.navRotModeRoll){
                case ctrl.FLY_ACRO:
                    bitSet(edMode_dirty, PID_ROLL_D_ACRO_BIT);
                    break;
                case ctrl.FLY_STAB:
                    bitSet(edMode_dirty, PID_ROLL_D_STAB_BIT);
                    break;
            }
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
    if(pendingUpdate_navRotRange){
        pendingUpdate_navRotRange = false;
        sendPacket_navRotRange();
        return;
    }
    if(pendingUpdate_navRotMode){
        pendingUpdate_navRotMode = false;
        sendPacket_navRotMode();
        return;
    }
    if (operatorButton_PS) {
      sendPacket_Ctrl();
    } else {
      sendPacket_Kill();
    }
}

void sendPacket_Ctrl() {
  ctrl.control_code = ctrl.NAV_CODE;
  /*
    ctrl.roll = (analogRead(7)-512)*64*-1;
    ctrl.pitch = (analogRead(6)-512)*64*-1;
    ctrl.yaw = (analogRead(8)-512)*64*-1;
    unsigned short temp = (max(analogRead(9),512)-512)*2;
    ctrl.throttle = temp*64;
  */
  stickLeft.update();
  stickRight.update();

  //Serial.printf("left x:%f y:%f\t right x:%f y:%f\n", stickLeft.x, stickLeft.y, stickRight.x, stickRight.y);
  ctrl.roll = int(stickRight.x * 64) * -1;
  ctrl.pitch = int(stickRight.y * 64) * -1;
  ctrl.yaw = int(stickLeft.x * 64) * -1;
  ctrl.throttle = max(int(stickLeft.y * 2 * 64) * -1, 0);

  ctrl.pack(buf);
  print("NAV ");
  print(ctrl.yaw); print(" ");
  print(ctrl.pitch); print(" ");
  print(ctrl.roll); print(" ");
  print(ctrl.throttle); print(" ");
  print(millis()); print("\n");
  radio.startTX(buf);
}
void sendPacket_navRotMode() {
  ctrl.control_code = ctrl.NAV_ROT_MODE_CODE;
  ctrl.yaw   = ctrl.navRotModeYaw;
  ctrl.pitch = ctrl.navRotModePitch;
  ctrl.roll  = ctrl.navRotModeRoll;
  ctrl.pack(buf);
  print("NRM "); // Nav Rotation Mode
  switch(ctrl.yaw){
      case ctrl.FLY_STAB:
        print("S ");
        break;
      case ctrl.FLY_ACRO:
        print("A ");
        break;
  }
  switch(ctrl.pitch){
      case ctrl.FLY_STAB:
        print("S ");
        break;
      case ctrl.FLY_ACRO:
        print("A ");
        break;
  }
  switch(ctrl.roll){
      case ctrl.FLY_STAB:
        print("S ");
        break;
      case ctrl.FLY_ACRO:
        print("A ");
        break;
  }
  print("\n");
  radio.startTX(buf);
}
void sendPacket_navRotRange() {
  ctrl.control_code = ctrl.NAV_ROT_RANGE_CODE;
  ctrl.yaw   = ctrl.navRotRangeYaw;
  ctrl.pitch = ctrl.navRotRangePitch;
  ctrl.roll  = ctrl.navRotRangeRoll;
  ctrl.pack(buf);
  print("NRR "); // Nav Rotation Range
  print(ctrl.yaw); print(" ");
  print(ctrl.pitch); print(" ");
  print(ctrl.roll); print(" ");
  print("\n");
  radio.startTX(buf);
}
void sendPacket_PID_yaw() {
  print("PID_YAW ");
  sendPacket_PID(ctrl.Kp_yaw, ctrl.Ki_yaw, ctrl.Kd_yaw, ctrl.PID_YAW_CODE);
}
void sendPacket_PID_pitch() {
  print("PID_PITCH ");
  sendPacket_PID(ctrl.Kp_pitch, ctrl.Ki_pitch, ctrl.Kd_pitch, ctrl.PID_PITCH_CODE);
}
void sendPacket_PID_roll() {
  print("PID_ROLL ");
  sendPacket_PID(ctrl.Kp_roll, ctrl.Ki_roll, ctrl.Kd_roll, ctrl.PID_ROLL_CODE);
}
void sendPacket_PID(float p, float i, float d, byte code) {
  ctrl.control_code = code;
  ctrl.yaw   = int(p * ctrl.pidScale);
  ctrl.pitch = int(i * ctrl.pidScale);
  ctrl.roll  = int(d * ctrl.pidScale);
  ctrl.pack(buf);
  print( (ctrl.yaw) / ctrl.pidScale); print(" ");
  print( (ctrl.pitch) / ctrl.pidScale); print(" ");
  print( (ctrl.roll) / ctrl.pidScale); print(" ");
  print("\n");
  radio.startTX(buf);
}
void sendPacket_Kill() {
  ctrl.control_code = ctrl.KILL_CODE;
  ctrl.pack(buf);
  //print("KILL_CODE \n");
  radio.startTX(buf);
}

void loadConfigFromMem(){
    print("loadConfigFromMem\n");
    // Nav Rotation Mode
    ctrl.navRotModeYaw = mem.read_RotMode_Yaw();
    ctrl.navRotModePitch = mem.read_RotMode_Pitch();
    ctrl.navRotModeRoll = mem.read_RotMode_Roll();
    print("Rotation Mode - yaw:"); print(ctrl.navRotModeYaw); print(" pitch:"); print(ctrl.navRotModePitch); print(" roll:"); print(ctrl.navRotModeRoll); print("\n");

    // Nav Rotation Range (byte) & PID settings per Nav Mode (unsigned short)
    switch (ctrl.navRotModeYaw) {
        case ctrl.FLY_STAB:
            ctrl.navRotRangeYaw = mem.read_RotRange_Yaw_Stab();
            ctrl.Kp_yaw = mem.read_PID_Yaw_P_Stab() / ctrl.pidScale;
            ctrl.Ki_yaw = mem.read_PID_Yaw_I_Stab() / ctrl.pidScale;
            ctrl.Kd_yaw = mem.read_PID_Yaw_D_Stab() / ctrl.pidScale;
            break;
        case ctrl.FLY_ACRO:
            ctrl.navRotRangeYaw = mem.read_RotRange_Yaw_Acro();
            ctrl.Kp_yaw = mem.read_PID_Yaw_P_Acro() / ctrl.pidScale;
            ctrl.Ki_yaw = mem.read_PID_Yaw_I_Acro() / ctrl.pidScale;
            ctrl.Kd_yaw = mem.read_PID_Yaw_D_Acro() / ctrl.pidScale;
            break;
    }
    switch (ctrl.navRotModePitch) {
        case ctrl.FLY_STAB:
            ctrl.navRotRangePitch = mem.read_RotRange_Pitch_Stab();
            ctrl.Kp_pitch = mem.read_PID_Pitch_P_Stab() / ctrl.pidScale;
            ctrl.Ki_pitch = mem.read_PID_Pitch_I_Stab() / ctrl.pidScale;
            ctrl.Kd_pitch = mem.read_PID_Pitch_D_Stab() / ctrl.pidScale;
            break;
        case ctrl.FLY_ACRO:
            ctrl.navRotRangePitch = mem.read_RotRange_Pitch_Acro();
            ctrl.Kp_pitch = mem.read_PID_Pitch_P_Acro() / ctrl.pidScale;
            ctrl.Ki_pitch = mem.read_PID_Pitch_I_Acro() / ctrl.pidScale;
            ctrl.Kd_pitch = mem.read_PID_Pitch_D_Acro() / ctrl.pidScale;
            break;
    }
    switch (ctrl.navRotModeRoll) {
        case ctrl.FLY_STAB:
            ctrl.navRotRangeRoll = mem.read_RotRange_Roll_Stab();
            ctrl.Kp_roll = mem.read_PID_Roll_P_Stab() / ctrl.pidScale;
            ctrl.Ki_roll = mem.read_PID_Roll_I_Stab() / ctrl.pidScale;
            ctrl.Kd_roll = mem.read_PID_Roll_D_Stab() / ctrl.pidScale;
            break;
        case ctrl.FLY_ACRO:
            ctrl.navRotRangeRoll = mem.read_RotRange_Roll_Acro();
            ctrl.Kp_roll = mem.read_PID_Roll_P_Acro() / ctrl.pidScale;
            ctrl.Ki_roll = mem.read_PID_Roll_I_Acro() / ctrl.pidScale;
            ctrl.Kd_roll = mem.read_PID_Roll_D_Acro() / ctrl.pidScale;
            break;
    }
    print("Rotation Range - yaw:"); print(ctrl.navRotRangeYaw); print(" pitch:"); print(ctrl.navRotRangePitch); print(" roll:"); print(ctrl.navRotRangeRoll); print("\n");
    print("PID Yaw   - P:"); print(ctrl.Kp_yaw); print(" I:"); print(ctrl.Ki_yaw); print(" D:"); print(ctrl.Kd_yaw); print("\n");
    print("PID Pitch - P:"); print(ctrl.Kp_pitch); print(" I:"); print(ctrl.Ki_pitch); print(" D:"); print(ctrl.Kd_pitch); print("\n");
    print("PID Roll  - P:"); print(ctrl.Kp_roll); print(" I:"); print(ctrl.Ki_roll); print(" D:"); print(ctrl.Kd_roll); print("\n");

};

void displayEdModeData(bool showData){
    display.clearDisplay();
    display.setCursor(0,0);
    switch (edMode){
        // Nav Mode
        case ED_ROTMODE_YAW:
            display.println("Flight Mode Yaw");
            if(showData){ displayRotNavMode(ctrl.navRotModeYaw); }
            break;
        case ED_ROTMODE_PITCH:
            display.println("Flight Mode Pitch");
            if(showData){ displayRotNavMode(ctrl.navRotModePitch); }
            break;
        case ED_ROTMODE_ROLL:
            display.println("Flight Mode Roll");
            if(showData){ displayRotNavMode(ctrl.navRotModeRoll); }
            break;

        // Nav Rotation Range
        case ED_ROTRANGE_YAW:
            display.println("RC Range Yaw");
            if(showData){
                display.println(ctrl.navRotRangeYaw);
                displayRotNavMode(ctrl.navRotModeYaw);
            }
            break;
        case ED_ROTRANGE_PITCH:
            display.println("RC Range Pitch");
            if(showData){
                display.println(ctrl.navRotRangePitch);
                displayRotNavMode(ctrl.navRotModePitch);
            }
            break;
        case ED_ROTRANGE_ROLL:
            display.println("RC Range Roll");
            if(showData){
                display.println(ctrl.navRotRangeRoll);
                displayRotNavMode(ctrl.navRotModeRoll);
            }
            break;

        // PID Yaw
        case ED_PID_YAW_P:
            display.println("PID Yaw P");
            if(showData){
                display.println(ctrl.Kp_yaw);
                displayRotNavMode(ctrl.navRotModeYaw);
            }
            break;
        case ED_PID_YAW_I:
            display.println("PID Yaw I");
            if(showData){
                display.println(ctrl.Ki_yaw);
                displayRotNavMode(ctrl.navRotModeYaw);
            }
            break;
        case ED_PID_YAW_D:
            display.println("PID Yaw D");
            if(showData){
                display.println(ctrl.Kd_yaw);
                displayRotNavMode(ctrl.navRotModeYaw);
            }
            break;

        // PID Pitch
        case ED_PID_PITCH_P:
            display.println("PID Pitch P");
            if(showData){
                display.println(ctrl.Kp_pitch);
                displayRotNavMode(ctrl.navRotModePitch);
            }
            break;
        case ED_PID_PITCH_I:
            display.println("PID Pitch I");
            if(showData){
                display.println(ctrl.Ki_pitch);
                displayRotNavMode(ctrl.navRotModePitch);
            }
            break;
        case ED_PID_PITCH_D:
            display.println("PID Pitch D");
            if(showData){
                display.println(ctrl.Kd_pitch);
                displayRotNavMode(ctrl.navRotModePitch);
            }
            break;

        // PID Roll
        case ED_PID_ROLL_P:
            display.println("PID Roll P");
            if(showData){
                display.println(ctrl.Kp_roll);
                displayRotNavMode(ctrl.navRotModeRoll);
            }
            break;
        case ED_PID_ROLL_I:
            display.println("PID Roll I");
            if(showData){
                display.println(ctrl.Ki_roll);
                displayRotNavMode(ctrl.navRotModeRoll);
            }
            break;
        case ED_PID_ROLL_D:
            display.println("PID Roll D");
            if(showData){
                display.println(ctrl.Kd_roll);
                displayRotNavMode(ctrl.navRotModeRoll);
            }
            break;
    }
    display.display();
}

void displayRotNavMode(byte mode){
    switch (mode){
        case ctrl.FLY_STAB:
            display.println("Stab");
            break;
        case ctrl.FLY_ACRO:
            display.println("Acro");
            break;
    }
}
