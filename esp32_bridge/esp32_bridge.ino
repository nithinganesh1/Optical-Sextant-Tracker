/* ESP32 DevKit V1 sun tracker — port of supplied sun_tracker_arduino.ino.
   Arduino Uno gimbal (A/Z motors + IMU) is INDEPENDENT, NOT controlled here.
   X/Y: Python USB serial 115200, same H/P/L/<dx> <dy>/READY/POS/OK/LIMIT/YRANGE protocol.
   Uno D1 TX -> (1k/2k resistor divider) -> GPIO16 RX2: IMU,roll,pitch,yaw.
   UART2 uses Core 0 task; X/Y + Python serial run on Core 1 task.
   Shield uses 3.3V-compatible control inputs/level buffers; switches require
   external pullups on GPIO34,35,39 (these have no internal pullups).
   WARNING: VERIFY MOTOR DIRECTIONS, SWITCHES, AND SAFE TRAVEL WITH LOAD DISENGAGED.
*/
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Exact GPIO mapping agreed previously
constexpr uint8_t X_STEP=25, X_DIR=26, Y_STEP=27, Y_DIR=14, EN_PIN=13;
constexpr uint8_t X_LIMIT=34, Y_LIMIT=35, Y_MAX_LIMIT=39;
constexpr uint8_t UNO_UART_RX=16;

// Switches NO-to-GND and each input pulled UP to 3.3V externally.
constexpr bool SWITCHES_NC=false; // false: LOW is pressed; true: HIGH is pressed
// Same signal-to-direction semantics as original Uno: LOW moves TOWARD home.
// If an axis runs away from home, change its TOWARD level, not its STEP pin.
constexpr uint8_t X_DIR_TOWARD_SWITCH=LOW;
constexpr uint8_t Y_DIR_TOWARD_SWITCH=LOW;

// ORIGINAL Uno had Y_SINGLE_SWITCH=true: Y home and far hit SAME electrical input.
// true: only GPIO35 is used for Y end detection, distinguish by motor position.
// false: GPIO35 is home, GPIO39 is a separate far switch.
constexpr bool Y_SINGLE_SWITCH=true;
constexpr bool HOME_ON_BOOT=false;  // PC will send 'H'; prevents double homing

// Reused original timing and distances. All maxima must suit the actual mechanism.
constexpr uint32_t STEP_HIGH_US=5;
constexpr uint32_t DIR_SETUP_US=15;
constexpr uint32_t MOVE_STEP_US=1000;
constexpr uint32_t START_STEP_US=2500;
constexpr uint32_t RAMP_DELTA_US=15;
constexpr uint32_t HOME_FAST_US=2000;
constexpr uint32_t HOME_SLOW_US=5000;
constexpr int32_t HOME_MAX_STEPS=12000;
constexpr int32_t HOME_BACKOFF_STEPS=100;
constexpr int32_t Y_DEFAULT_TRAVEL_STEPS=3200;
constexpr int32_t Y_RANGE_SEARCH_STEPS=3600;
constexpr int32_t X_SOFT_MAX_STEPS=1067;  // (60 - -60) / 0.1125 deg/step = 1067 steps to the surface-level pose
constexpr int32_t Y_SOFT_MAX_STEPS=3200;
constexpr int32_t MAX_MOVE_STEPS=20000;
constexpr uint32_t IMU_STALE_MS=1200;
// Check every asserted switch transition for a continuously stable 8 ms.
// Release is also verified; the value in L remains an instantaneous status.
constexpr uint32_t HOME_DEBOUNCE_US=8000;
constexpr int32_t HOME_CLEARANCE_STEPS=100;
// Set false while verifying motion; only after successful home tests enable Y range scan.
constexpr bool MEASURE_Y_RANGE_ON_HOME=false;


HardwareSerial UnoIMU(2);
portMUX_TYPE imuLock=portMUX_INITIALIZER_UNLOCKED;
float imuRoll=0,imuPitch=0,imuYaw=0;
uint32_t imuMs=0;
bool imuOK=false;
int32_t posX=0,posY=0,yRange=0;
bool homed=false;

inline bool limitPressed(uint8_t pin){return digitalRead(pin)==(SWITCHES_NC?HIGH:LOW);}
// Only accept an active or inactive switch state if it persists continuously.
// This is deliberately used in homing, not every pulse in normal motion.
bool confirmedSwitch(uint8_t pin,bool target){
  if(limitPressed(pin)!=target)return false;
  uint32_t t=micros();
  while((uint32_t)(micros()-t)<HOME_DEBOUNCE_US){
    if(limitPressed(pin)!=target)return false;
    delayMicroseconds(100);
  }
  return true;
}
inline void pulse(uint8_t pin){digitalWrite(pin,HIGH);delayMicroseconds(STEP_HIGH_US);digitalWrite(pin,LOW);}
inline void intervalWait(uint32_t us){delayMicroseconds(us);}
void sendLimits(){
  Serial.print("LIM ");Serial.print(limitPressed(X_LIMIT)?1:0);Serial.print(' ');
  Serial.print(limitPressed(Y_LIMIT)?1:0);Serial.print(' ');
  Serial.println(limitPressed(yFarPin())?1:0);
}
void sendPos(){Serial.printf("POS %ld %ld\n",(long)posX,(long)posY);}
void sendOK(){Serial.printf("OK %ld %ld\n",(long)posX,(long)posY);}
void motorDir(uint8_t pin,uint8_t level){digitalWrite(pin,level);delayMicroseconds(DIR_SETUP_US);}

// Stepping with a stable switch confirmation. Returning count does NOT imply
// encoder-confirmed physical motion; DIR output and mechanical movement must be verified.
int32_t seekSwitch(const char* axis,uint8_t step,uint8_t dir,uint8_t sw,
                   uint8_t towardLevel,bool seekPressed,uint32_t stepUs,
                   int32_t maxSteps,const char* phase){
  uint8_t level=seekPressed?towardLevel:(towardLevel==LOW?HIGH:LOW);
  motorDir(dir,level);
  Serial.printf("HOME %s %s DIR_GPIO %u LEVEL %u TARGET %s\n",axis,phase,
                (unsigned)dir,(unsigned)level,seekPressed?"PRESSED":"RELEASED");
  for(int32_t n=0;n<=maxSteps;n++){
    if(confirmedSwitch(sw,seekPressed))return n;
    if(n==maxSteps)break;
    pulse(step);intervalWait(stepUs);
    if((n&31)==0)yield();
  }
  return -1;
}

// Used only for the optional Y-range return sequence.
int32_t runUntilSwitch(uint8_t step,uint8_t dir,uint8_t sw,uint8_t towardLevel,
                       bool toward,uint32_t stepUs,int32_t maxSteps){
  return seekSwitch("Y",step,dir,sw,towardLevel,toward,stepUs,maxSteps,"RANGE_RELEASE");
}

bool homeAxis(const char* axis,uint8_t step,uint8_t dir,uint8_t sw,uint8_t towardLevel){
  Serial.printf("HOME %s START RAW=%u PRESSED=%u\n",axis,
                (unsigned)digitalRead(sw),(unsigned)limitPressed(sw));
  // If switch is already pressed at entry, first get off it.
  if(confirmedSwitch(sw,true)){
    int32_t n=seekSwitch(axis,step,dir,sw,towardLevel,false,HOME_SLOW_US,
                         HOME_BACKOFF_STEPS*4,"INITIAL_RELEASE");
    if(n<0){Serial.printf("HOME %s FAIL_INITIAL_RELEASE\n",axis);return false;}
    Serial.printf("HOME %s INITIAL_RELEASE_STEPS %ld\n",axis,(long)n);
  }
  int32_t hit=seekSwitch(axis,step,dir,sw,towardLevel,true,HOME_FAST_US,
                         HOME_MAX_STEPS,"SEEK_1");
  if(hit<0){Serial.printf("HOME %s FAIL_SEEK_1\n",axis);return false;}
  Serial.printf("HOME %s HIT_1_STEPS %ld\n",axis,(long)hit);
  int32_t released=seekSwitch(axis,step,dir,sw,towardLevel,false,HOME_SLOW_US,
                              HOME_BACKOFF_STEPS*4,"RELEASE_1");
  if(released<0){Serial.printf("HOME %s FAIL_RELEASE_1\n",axis);return false;}
  Serial.printf("HOME %s RELEASED_1_STEPS %ld\n",axis,(long)released);
  // Leave the contact by a known, fixed clearance: a brief low glitch does
  // not cancel this travel. Abort only if pressed stays stable after clearance.
  motorDir(dir,towardLevel==LOW?HIGH:LOW);
  for(int32_t i=0;i<HOME_CLEARANCE_STEPS;i++){
    pulse(step);intervalWait(HOME_SLOW_US);if((i&31)==0)yield();
  }
  Serial.printf("HOME %s CLEARANCE_STEPS %ld RAW=%u\n",axis,
                (long)HOME_CLEARANCE_STEPS,(unsigned)digitalRead(sw));
  if(confirmedSwitch(sw,true)){
    Serial.printf("HOME %s FAIL_STILL_PRESSED_AFTER_CLEARANCE\n",axis);
    return false;
  }
  // Budget returns all commanded reverse travel PLUS reasonable hysteresis.
  int32_t budget=released+HOME_CLEARANCE_STEPS+HOME_BACKOFF_STEPS;
  Serial.printf("HOME %s SEEK_2_BUDGET %ld\n",axis,(long)budget);
  int32_t second=seekSwitch(axis,step,dir,sw,towardLevel,true,HOME_SLOW_US,
                            budget,"SEEK_2");
  if(second<0){
    Serial.printf("HOME %s FAIL_SEEK_2 RAW=%u (verify physical DIR reversal)\n",
                  axis,(unsigned)digitalRead(sw));
    return false;
  }
  Serial.printf("HOME %s HIT_2_STEPS %ld\n",axis,(long)second);
  Serial.printf("HOME %s OK\n",axis);
  return true;
}

inline uint8_t yFarPin(){return Y_SINGLE_SWITCH?Y_LIMIT:Y_MAX_LIMIT;}
inline int32_t yMid(){return (yRange>0?yRange:Y_DEFAULT_TRAVEL_STEPS)/2;}

bool yHomeHit(bool movingToward){
  if(!movingToward)return false;
  return Y_SINGLE_SWITCH?(limitPressed(Y_LIMIT)&&posY<=yMid()):limitPressed(Y_LIMIT);
}
bool yFarHit(bool movingToward){
  if(movingToward)return false;
  return Y_SINGLE_SWITCH?(limitPressed(Y_LIMIT)&&posY>yMid()):limitPressed(Y_MAX_LIMIT);
}

bool measureYRange(){
  yRange=0;
  Serial.println("HOME Y RANGE_SEARCH");
  motorDir(Y_DIR,Y_DIR_TOWARD_SWITCH==LOW?HIGH:LOW);
  int32_t n=0;
  // First leave the home contact; switch may be same at both ends.
  if(Y_SINGLE_SWITCH){
    while(n<Y_RANGE_SEARCH_STEPS && limitPressed(Y_LIMIT)){
      pulse(Y_STEP);intervalWait(HOME_FAST_US);n++;if((n&31)==0)yield();
    }
    if(limitPressed(Y_LIMIT)){Serial.println("ERR Y_RANGE_HOME_STUCK");return false;}
  }
  while(n<Y_RANGE_SEARCH_STEPS && !limitPressed(yFarPin())){
    pulse(Y_STEP);intervalWait(HOME_FAST_US);n++;if((n&31)==0)yield();
  }
  if(!limitPressed(yFarPin())||n<=100){Serial.println("ERR Y_FAR_SWITCH_NOT_FOUND");return false;}
  yRange=n;
  Serial.printf("YRANGE %ld\n",(long)yRange);
  posY=yRange;
  return true;
}

bool homeAll(){
  homed=false;
  Serial.println("HOMING");
  Serial.print("HOME SWITCHES ");Serial.print(limitPressed(X_LIMIT));Serial.print(' ');
  Serial.print(limitPressed(Y_LIMIT));Serial.print(' ');Serial.println(limitPressed(yFarPin()));
  digitalWrite(EN_PIN,LOW);
  if(!homeAxis("X",X_STEP,X_DIR,X_LIMIT,X_DIR_TOWARD_SWITCH)){
    Serial.println("ERR X_HOME_FAILED");digitalWrite(EN_PIN,HIGH);return false;
  }
  posX=0;
  if(!homeAxis("Y",Y_STEP,Y_DIR,Y_LIMIT,Y_DIR_TOWARD_SWITCH)){
    Serial.println("ERR Y_HOME_FAILED");digitalWrite(EN_PIN,HIGH);return false;
  }
  posY=0;
  if(MEASURE_Y_RANGE_ON_HOME){
    if(!measureYRange()){
      digitalWrite(EN_PIN,HIGH);return false;
    }
    // Old Uno drives off shared far switch toward home before seeking home again.
    if(Y_SINGLE_SWITCH && limitPressed(Y_LIMIT)){
      int32_t leave=runUntilSwitch(Y_STEP,Y_DIR,Y_LIMIT,Y_DIR_TOWARD_SWITCH,true,
                                   HOME_SLOW_US,HOME_BACKOFF_STEPS*4);
      if(leave<0){Serial.println("ERR Y_FAR_RELEASE_FAILED");digitalWrite(EN_PIN,HIGH);return false;}
      motorDir(Y_DIR,Y_DIR_TOWARD_SWITCH);
      for(int32_t i=0;i<HOME_BACKOFF_STEPS;i++){
        pulse(Y_STEP);intervalWait(HOME_SLOW_US);if((i&31)==0)yield();
      }
    }
    if(!homeAxis("Y",Y_STEP,Y_DIR,Y_LIMIT,Y_DIR_TOWARD_SWITCH)){
      Serial.println("ERR Y_RETURN_HOME_FAILED");digitalWrite(EN_PIN,HIGH);return false;
    }
    posY=0;
  }
  homed=true;
  Serial.println("READY");
  return true;
}

void moveRelative(int32_t dx,int32_t dy){
  if(!homed){Serial.println("ERR NOT_HOMED");return;}
  if(labs(dx)>MAX_MOVE_STEPS||labs(dy)>MAX_MOVE_STEPS){Serial.println("ERR MOVE_TOO_LARGE");return;}
  int32_t yMax=(yRange>0)?min(Y_SOFT_MAX_STEPS,yRange):Y_SOFT_MAX_STEPS;
  int32_t tx=constrain(posX+dx,0L,(long)X_SOFT_MAX_STEPS);
  int32_t ty=constrain(posY+dy,0L,(long)yMax);
  int32_t mx=tx-posX,my=ty-posY;
  uint32_t nx=labs(mx),ny=labs(my),total=max(nx,ny);
  if(!total){sendOK();return;}
  bool xtoward=mx<0,ytoward=my<0;
  motorDir(X_DIR,xtoward?X_DIR_TOWARD_SWITCH:!X_DIR_TOWARD_SWITCH);
  motorDir(Y_DIR,ytoward?Y_DIR_TOWARD_SWITCH:!Y_DIR_TOWARD_SWITCH);
  uint32_t ex=0,ey=0;bool xBlocked=false,yBlocked=false;
  uint32_t interval=START_STEP_US;
  for(uint32_t i=0;i<total;i++){
    ex+=nx;ey+=ny;
    if(nx && !xBlocked && ex>=total){
      ex-=total;
      if(xtoward&&limitPressed(X_LIMIT)){Serial.println("LIMIT X");posX=0;xBlocked=true;}
      else{pulse(X_STEP);posX+=xtoward?-1:1;}
    }
    if(ny && !yBlocked && ey>=total){
      ey-=total;
      if(yHomeHit(ytoward)){Serial.println("LIMIT Y");posY=0;yBlocked=true;}
      else if(yFarHit(ytoward)){Serial.println("LIMIT YMAX");if(yRange>0)posY=yRange;yBlocked=true;}
      else{pulse(Y_STEP);posY+=ytoward?-1:1;}
    }
    intervalWait(interval);
    if(interval>MOVE_STEP_US){interval=(interval-MOVE_STEP_US>RAMP_DELTA_US)?interval-RAMP_DELTA_US:MOVE_STEP_US;}
    if((i&31)==0)yield();
    if((nx==0||xBlocked)&&(ny==0||yBlocked))break;
  }
  sendOK();
}

void saveIMU(const char* s){
  if(strcmp(s,"IMU,ERR")==0){portENTER_CRITICAL(&imuLock);imuOK=false;portEXIT_CRITICAL(&imuLock);return;}
  float r,p,y;char extra;
  if(sscanf(s,"IMU,%f,%f,%f%c",&r,&p,&y,&extra)!=3 || !isfinite(r)||!isfinite(p)||!isfinite(y))return;
  portENTER_CRITICAL(&imuLock);
  imuRoll=r;imuPitch=p;imuYaw=y;imuMs=millis();imuOK=true;
  portEXIT_CRITICAL(&imuLock);
}
void imuTask(void*){
  char buf[70];size_t len=0;
  for(;;){
    while(UnoIMU.available()){
      char c=(char)UnoIMU.read();
      if(c=='\r')continue;
      if(c=='\n'){buf[len]=0;if(len)saveIMU(buf);len=0;}
      else if(len<sizeof(buf)-1)buf[len++]=c;else len=0;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
void sendIMU(){
  float r,p,y;uint32_t t;bool ok;
  portENTER_CRITICAL(&imuLock);r=imuRoll;p=imuPitch;y=imuYaw;t=imuMs;ok=imuOK;portEXIT_CRITICAL(&imuLock);
  if(!ok||(uint32_t)(millis()-t)>IMU_STALE_MS){Serial.println("IMU,ERR");return;}
  Serial.printf("IMU,%.2f,%.2f,%.2f\n",r,p,y);
}
void sendLegacyGS(){
  float r;uint32_t t;bool ok;
  portENTER_CRITICAL(&imuLock);r=imuRoll;t=imuMs;ok=imuOK;portEXIT_CRITICAL(&imuLock);
  bool fresh=ok && (uint32_t)(millis()-t)<=IMU_STALE_MS;
  // Old dashboard parser expects GIMBAL <active> <imuOK> <roll> <rollRate> <faults>
  Serial.printf("GIMBAL 1 %d %.2f 0 0\n",fresh?1:0,fresh?r:0.0f);
}
void command(const char* cmd){
  if(strcmp(cmd,"H")==0){homeAll();return;}
  if(strcmp(cmd,"P")==0){sendPos();return;}
  if(strcmp(cmd,"L")==0){sendLimits();return;}
  if(strcmp(cmd,"I")==0){sendIMU();return;}
  if(strcmp(cmd,"GS")==0){sendLegacyGS();sendIMU();return;}
  if(strcmp(cmd,"GP1")==0||strcmp(cmd,"GP0")==0)return;
  if(strcmp(cmd,"G0")==0||strcmp(cmd,"G1")==0){Serial.println("ERR GIMBAL_AUTONOMOUS_ON_UNO");return;}
  long dx,dy;char excess;
  if(sscanf(cmd,"%ld %ld %c",&dx,&dy,&excess)==2){moveRelative(dx,dy);return;}
  Serial.println("ERR BAD_COMMAND");
}
void trackerTask(void*){
  // BOOT can be missed on port open; PC will issue H itself.
  Serial.println("BOOT");
  if(HOME_ON_BOOT)homeAll();else Serial.println("READY (not homed)");
  char line[64];size_t n=0;
  for(;;){
    while(Serial.available()){
      char c=(char)Serial.read();
      if(c=='\r'||c=='\n'){
        if(n){line[n]=0;command(line);n=0;}
      }else if(n<sizeof(line)-1){line[n++]=c;}
      else n=0;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void setup(){
  Serial.begin(115200);
  pinMode(X_STEP,OUTPUT);pinMode(X_DIR,OUTPUT);pinMode(Y_STEP,OUTPUT);pinMode(Y_DIR,OUTPUT);
  pinMode(EN_PIN,OUTPUT);
  digitalWrite(X_STEP,LOW);digitalWrite(Y_STEP,LOW);digitalWrite(EN_PIN,HIGH);
  pinMode(X_LIMIT,INPUT);pinMode(Y_LIMIT,INPUT);pinMode(Y_MAX_LIMIT,INPUT);
  UnoIMU.begin(115200,SERIAL_8N1,UNO_UART_RX,-1);
  xTaskCreatePinnedToCore(imuTask,"IMU_receiver",3072,nullptr,1,nullptr,0);
  xTaskCreatePinnedToCore(trackerTask,"XY_and_USB",8192,nullptr,1,nullptr,1);
}
void loop(){vTaskDelay(pdMS_TO_TICKS(1000));}