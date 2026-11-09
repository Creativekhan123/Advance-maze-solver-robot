#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include "RouteMap.h"

using route::Command;
// V05_Left_Hand calibration and wiring: deliberately retained.
constexpr int AIN1=14, AIN2=27, PWMA=23, BIN1=19, BIN2=18, PWMB=5;
constexpr int left_encoder_pin=33, right_encoder_pin=34;
constexpr int F_Sensor=13, R_Sensor=26, L_Sensor=25;
constexpr int motorTurnSpeed=150, motorForwardSpeed=200, rest=150;
constexpr int turn_encoder_value=520, each_step_encoder_value=5, extraSteps=800;
constexpr int distance_threshold=180, too_close_distance=7;
constexpr bool autoMode=true;
constexpr uint32_t STALL_MS=700, MOTION_TIMEOUT_MS=12000, SENSOR_TIMEOUT_MS=300;
constexpr int FRONT_STOP_MM=35; // independent collision guard, not a turn threshold
constexpr uint32_t STORAGE_MAGIC=0x365A414D, STORAGE_VERSION=1;

BluetoothSerial SerialBT;
Adafruit_VL53L0X F_lox, L_lox, R_lox;
Preferences preferences;
enum Mode { IDLE, LEARN, REPLAY };
Mode mode=IDLE;
bool stopRequested=false, finishRequested=false, faulted=false, hardwareReady=false;
bool cruiseContext=false, trackingCruise=false, storageReady=false;
volatile uint32_t leftTicks=0, rightTicks=0;
portMUX_TYPE encoderMux=portMUX_INITIALIZER_UNLOCKED;
uint32_t accountedLeft=0, accountedRight=0;
int signLeft=0, signRight=0;
route::Pose pose={0,0,0};
float wheelTravel=0, pendingCruise=0, centreTravel=0;
int f=0,l=0,r=0;
uint32_t sensorTime[3]={};
uint32_t sensorSequence[3]={};
uint8_t stableMask=0, lastMask=255, stableSamples=0;
uint32_t lastMaskSequence[3]={};
route::Graph graph;
Command recorded[route::MAX_COMMANDS];
uint16_t recordedCount=0, edgeFirst=0;
int currentNode=-1;
float edgeStartTravel=0, lastCheckpointTravel=0;
bool eventLatched=false;
uint8_t junctionWorldMask=0;
bool checkpoint(bool finalPoint=false, int maskOverride=-1);

struct SavedRoute {
  uint32_t magic, version, count, hash;
  Command commands[route::MAX_COMMANDS];
};
SavedRoute saved={};
bool savedValid=false;

void logMessage(const char *message) { Serial.println(message); SerialBT.println(message); }
bool running() { return mode!=IDLE && !stopRequested && !faulted; }
void IRAM_ATTR leftEncoderISR() {
  portENTER_CRITICAL_ISR(&encoderMux); ++leftTicks; portEXIT_CRITICAL_ISR(&encoderMux);
}
void IRAM_ATTR rightEncoderISR() {
  portENTER_CRITICAL_ISR(&encoderMux); ++rightTicks; portEXIT_CRITICAL_ISR(&encoderMux);
}
void ticks(uint32_t &a,uint32_t &b) {
  portENTER_CRITICAL(&encoderMux); a=leftTicks; b=rightTicks; portEXIT_CRITICAL(&encoderMux);
}
void brakeA() { digitalWrite(AIN1,HIGH); digitalWrite(AIN2,HIGH); analogWrite(PWMA,255); }
void brakeB() { digitalWrite(BIN1,HIGH); digitalWrite(BIN2,HIGH); analogWrite(PWMB,255); }
void brakeBothMotors() { brakeA(); brakeB(); }
void fail(const char *why) {
  if (!faulted) logMessage(why);
  faulted=true; stopRequested=true; brakeBothMotors();
}
void accountMotion() {
  uint32_t a,b; ticks(a,b);
  float dl=float(a-accountedLeft)*signLeft, dr=float(b-accountedRight)*signRight;
  accountedLeft=a; accountedRight=b;
  route::integrate(pose,dl,dr);
  wheelTravel+=(fabsf(dl)+fabsf(dr))*0.5f;
  centreTravel+=(dl+dr)*0.5f;
  if (mode==LEARN && trackingCruise) pendingCruise+=(dl+dr)*0.5f;
}
bool append(Command c) {
  if (mode!=LEARN || faulted) return false;
  if (recordedCount==route::MAX_COMMANDS) { fail("FAULT: movement log full; previous saved route retained."); return false; }
  recorded[recordedCount++]=c; return true;
}
void flushCruise() {
  if (pendingCruise>=1) {
    append({(uint32_t)lroundf(pendingCruise),0,0,0,route::CRUISE,0,0});
  }
  pendingCruise=0;
}
void serviceCommands() {
  // Starts are handled only by idle loop. During movement only stop/status are accepted.
  for (Stream *stream : {static_cast<Stream*>(&Serial),static_cast<Stream*>(&SerialBT)}) {
    int budget=32;
    while (budget-- && stream->available()) {
      char c=stream->read();
      if (c=='2' || c=='0' || c=='s' || c=='S') {
        if (!stopRequested) finishRequested=(c=='2' && mode==LEARN);
        if (c!='2') finishRequested=false;
        stopRequested=true; brakeBothMotors();
      } else if (c=='1' || c=='3' || c=='C' || c=='c') {
        logMessage("Busy: use 2 to finish, or 0 to abort; start/clear commands ignored.");
      }
    }
  }
}
void sampleSensor(Adafruit_VL53L0X &sensor,int &value,int index) {
  if (!sensor.isRangeComplete()) return;
  uint16_t mm=sensor.readRangeResult();
  if (sensor.readRangeStatus()==0 && mm!=0xffff && mm>0) {
    value=mm; sensorTime[index]=millis(); ++sensorSequence[index];
  }
  // Invalid data never becomes a fictitious opening; old data expires below.
}
bool sensorsFresh() {
  uint32_t now=millis();
  for (int i=0;i<3;++i) if (!sensorSequence[i] || now-sensorTime[i]>SENSOR_TIMEOUT_MS) return false;
  return true;
}
uint8_t wallMask() { return (l>180?1:0)|(f>150?2:0)|(r>180?4:0); }
void pollSensors() {
  static uint32_t previous=0;
  if (millis()-previous<5) return;
  previous=millis();
  sampleSensor(L_lox,l,0); sampleSensor(F_lox,f,1); sampleSensor(R_lox,r,2);
  bool allNew=true;
  for (int i=0;i<3;++i) allNew &= sensorSequence[i]!=lastMaskSequence[i];
  if (allNew && sensorsFresh()) {
    for (int i=0;i<3;++i) lastMaskSequence[i]=sensorSequence[i];
    uint8_t mask=wallMask();
    stableSamples=mask==lastMask ? (uint8_t)min(255,int(stableSamples)+1) : 1;
    lastMask=mask;
    if (stableSamples>=3) stableMask=mask;
  }
  if (running() && !sensorsFresh()) fail("FAULT: distance sensor invalid/stale; stopped without saving.");
  if (running() && signLeft>0 && signRight>0 && f<FRONT_STOP_MM)
    fail("FAULT: front clearance below 35 mm; stopped without saving.");
}
void responsivePause(uint32_t ms) {
  uint32_t start=millis();
  while (millis()-start<ms && running()) {
    serviceCommands(); accountMotion(); pollSensors(); delay(1);
  }
}
void stopAndSettle() {
  brakeBothMotors();
  // Include braking pulses in odometry and the recorded finish position.
  uint32_t start=millis();
  while (millis()-start<60) {
    accountMotion(); serviceCommands(); pollSensors(); delay(1);
  }
  accountMotion(); signLeft=signRight=0;
}
void driveWheel(int speed,bool left) {
  int in1=left?AIN1:BIN1,in2=left?AIN2:BIN2,pwm=left?PWMA:PWMB;
  if (!speed) { if(left) brakeA(); else brakeB(); return; }
  digitalWrite(in1,speed>0?HIGH:LOW); digitalWrite(in2,speed>0?LOW:HIGH);
  analogWrite(pwm,abs(speed));
}
void move(int speedA,uint32_t targetA,int speedB,uint32_t targetB,bool glide=false) {
  if (!running()) return;
  if (!glide) { stopAndSettle(); responsivePause(rest-60); }
  if (!running()) return;
  if (!cruiseContext) { flushCruise(); trackingCruise=false; }
  else trackingCruise=true;
  bool cardinalTurn=mode==LEARN && !cruiseContext && junctionWorldMask &&
    speedA*speedB<0 && targetA==targetB && (targetA==520 || targetA==1040);
  if(cardinalTurn && !checkpoint(false,0x80|junctionWorldMask)) return;
  accountMotion();
  uint32_t startA,startB; ticks(startA,startB);
  signLeft=targetA?(speedA>0?1:-1):0; signRight=targetB?(speedB>0?1:-1):0;
  driveWheel(targetA?speedA:0,true); driveWheel(targetB?speedB:0,false);
  uint32_t started=millis(),lastProgressA=started,lastProgressB=started,oldA=startA,oldB=startB;
  while (running()) {
    serviceCommands(); accountMotion(); pollSensors();
    uint32_t a,b; ticks(a,b); uint32_t now=millis();
    if (a!=oldA) { oldA=a; lastProgressA=now; }
    if (b!=oldB) { oldB=b; lastProgressB=now; }
    bool doneA=a-startA>=targetA,doneB=b-startB>=targetB;
    if (!glide) { if(doneA) brakeA(); if(doneB) brakeB(); }
    if (doneA && doneB) break;
    if ((!doneA && now-lastProgressA>STALL_MS) || (!doneB && now-lastProgressB>STALL_MS)) {
      fail("FAULT: encoder stalled; stopped without saving."); break;
    }
    if (now-started>MOTION_TIMEOUT_MS) { fail("FAULT: movement timeout."); break; }
    delay(1);
  }
  if (!glide || !running()) stopAndSettle();
  if (mode==LEARN && !cruiseContext && !faulted) {
    uint32_t a,b; ticks(a,b);
    uint32_t executedA=min(a-startA,targetA), executedB=min(b-startB,targetB);
    // Preserve V5's commanded counts; do not add overshoot again on every replay.
    if (executedA || executedB) append({executedA,executedB,(int16_t)speedA,(int16_t)speedB,route::RAW,0,0});
  }
  if(cardinalTurn && running()) checkpoint(false,0x80|junctionWorldMask);
}
void motorA(int speed,int target) { move(speed,target,0,0); }
void motorB(int speed,int target) { move(0,0,speed,target); }
void moveBothMotors(int a,int ta,int b,int tb) { move(a,ta,b,tb); }
void moveBothMotorsWithoutBreak(int a,int ta,int b,int tb) { move(a,ta,b,tb,true); }

// V5 steering sequence; cruise is saved as distance, not a list of steering
// twitches, so replay uses fresh wall readings to steer again.
void keep_center() {
  if (!running()) return;
  cruiseContext=true;
  if (r < distance_threshold/2.8 || l < distance_threshold/2.8) {
    if (r>l) { motorA(255,each_step_encoder_value); motorB(-100,each_step_encoder_value); }
    else if (l>r) { motorA(-100,each_step_encoder_value); motorB(255,each_step_encoder_value); }
    else moveBothMotorsWithoutBreak(motorForwardSpeed,each_step_encoder_value,motorForwardSpeed,each_step_encoder_value);
  } else moveBothMotorsWithoutBreak(motorForwardSpeed,each_step_encoder_value,motorForwardSpeed,each_step_encoder_value);
  cruiseContext=false;
}

void autonomus(); // V5 maneuver bodies below

bool settleSignature() {
  stopAndSettle(); stableSamples=0; lastMask=255;
  uint32_t start=millis();
  while (running() && millis()-start<800) {
    serviceCommands(); pollSensors(); accountMotion();
    if (stableSamples>=3 && sensorsFresh()) return true;
    delay(1);
  }
  if (running()) fail("FAULT: unstable junction readings.");
  return false;
}
bool checkpoint(bool finalPoint, int maskOverride) {
  flushCruise();
  if (faulted) return false;
  uint8_t mask=finalPoint?255:(maskOverride>=0?(uint8_t)maskOverride:stableMask);
  int oldCount=graph.nodeCount;
  int node=graph.landmark(pose,mask,wheelTravel,!finalPoint);
  if (node<0) { fail("FAULT: map node capacity reached."); return false; }
  if (currentNode>=0 && !graph.connect(currentNode,node,edgeFirst,recordedCount-edgeFirst,wheelTravel-edgeStartTravel)) {
    fail("FAULT: map edge capacity or empty segment."); return false;
  }
  if (node<oldCount) logMessage("Map: matched a previously visited pose (estimated loop closure).");
  currentNode=node; edgeFirst=recordedCount; edgeStartTravel=wheelTravel;
  lastCheckpointTravel=wheelTravel;
  // Centre signatures are world-frame topology captured before alignment;
  // only actual sensor checkpoints become replay CHECK commands.
  if (!finalPoint && maskOverride<0) append({0,0,0,0,route::CHECK,mask,0});
  return !faulted;
}
bool validateSaved(size_t bytes) {
  if (saved.magic!=STORAGE_MAGIC || saved.version!=STORAGE_VERSION || !saved.count ||
      saved.count>route::MAX_COMMANDS || bytes!=16+saved.count*sizeof(Command)) return false;
  if (route::checksum(saved.commands,saved.count*sizeof(Command))!=saved.hash) return false;
  for(uint32_t i=0;i<saved.count;++i) if(!route::validCommand(saved.commands[i])) return false;
  return true;
}
void loadSaved() {
  savedValid=false;
  if(!storageReady) return;
  size_t bytes=preferences.getBytesLength("route");
  if(bytes<16 || bytes>sizeof(saved)) return;
  if(preferences.getBytes("route",&saved,bytes)==bytes) savedValid=validateSaved(bytes);
}
void saveFinished() {
  if (!recordedCount || wheelTravel<1) { logMessage("No movement to save; previous route retained."); return; }
  if (!checkpoint(true)) return;
  static uint16_t path[route::MAX_NODES];
  int count=graph.shortest(0,currentNode,path,route::MAX_NODES);
  if(count<0) { fail("FAULT: no connected route to finish."); return; }
  saved.count=0;
  float cost=0;
  for(int i=0;i<count;++i) {
    const route::Edge &edge=graph.edges[path[i]]; cost+=edge.cost;
    if(edge.turnQuarters) {
      if(saved.count==route::MAX_COMMANDS) { fail("FAULT: solution too large."); loadSaved(); return; }
      int q=edge.turnQuarters, speed=q>0?-motorTurnSpeed:motorTurnSpeed;
      saved.commands[saved.count++]={(uint32_t)(abs(q)*520),(uint32_t)(abs(q)*520),
        (int16_t)speed,(int16_t)-speed,route::RAW,0,0};
    }
    for(int j=0;j<edge.count;++j) {
      if(saved.count==route::MAX_COMMANDS) { fail("FAULT: solution too large."); loadSaved(); return; }
      saved.commands[saved.count++]=recorded[edge.first+j];
    }
  }
  saved.magic=STORAGE_MAGIC; saved.version=STORAGE_VERSION;
  saved.hash=route::checksum(saved.commands,saved.count*sizeof(Command));
  size_t bytes=16+saved.count*sizeof(Command);
  if(!storageReady || !validateSaved(bytes) || preferences.putBytes("route",&saved,bytes)!=bytes) {
    logMessage("SAVE FAILED: previous flash route retained; check NVS capacity."); loadSaved(); return;
  }
  loadSaved();
  char message[160];
  snprintf(message,sizeof(message),"Saved %lu commands; map %u poses/%u edges; wheel effort %.0f -> %.0f counts. Use 3 after replacing at start.",
    (unsigned long)saved.count,graph.nodeCount,graph.edgeCount,wheelTravel,cost);
  logMessage(savedValid?message:"SAVE verification failed; replay disabled.");
}
void resetMotion() {
  stopRequested=finishRequested=faulted=false;
  signLeft=signRight=0; trackingCruise=cruiseContext=false;
  ticks(accountedLeft,accountedRight); pose={0,0,0}; wheelTravel=pendingCruise=centreTravel=0;
  junctionWorldMask=0;
  eventLatched=false;
}
void exploreStep() {
  pollSensors(); accountMotion();
  if(!running()) return;
  // Use V5's actual action thresholds, including its asymmetric right-only
  // threshold. An opening seen too early must not latch a turn that never ran.
  bool leftOpen=l>180, rightOpen=r>180;
  bool junction=(!leftOpen && !rightOpen && f<150) ||
    (leftOpen && !rightOpen && (f<150 || f>150)) ||
    (rightOpen && !leftOpen && (f<120 || f>180)) ||
    (leftOpen && rightOpen && (f<120 || f>120));
  if(!leftOpen && !rightOpen && f>=150) eventLatched=false;
  if(junction && !eventLatched) {
    if(!settleSignature()) return;
    if(!checkpoint()) return;
    eventLatched=true;
    int heading=(int)lroundf(pose.yaw/(route::PI_F/2));
    junctionWorldMask=0;
    if(fabsf(route::angle(pose.yaw-heading*route::PI_F/2))<route::MATCH_ANGLE) {
      auto directionBit=[heading](int relative) { return (uint8_t)(1u<<((heading+relative+8)%4)); };
      junctionWorldMask=directionBit(2); // the arrival passage is known open
      if(stableMask&1) junctionWorldMask|=directionBit(1);
      if(stableMask&2) junctionWorldMask|=directionBit(0);
      if(stableMask&4) junctionWorldMask|=directionBit(-1);
    }
    autonomus();
    junctionWorldMask=0;
  } else if (junction && eventLatched && f<150) {
    // Never drive blindly into a blocked front while suppressing duplicate events.
    fail("FAULT: junction did not clear after maneuver; inspect alignment.");
  } else {
    keep_center();
  }
}
void replayCruise(uint32_t target) {
  float initialTravel=wheelTravel, initialCentre=centreTravel, lastProgress=centreTravel;
  // Distance is signed centre travel, not turn effort. Accumulate per iteration
  // so steering pulses don't count as forward progress.
  uint32_t timeoutStart=millis();
  while(running() && centreTravel-initialCentre<target) {
    serviceCommands(); accountMotion(); pollSensors();
    if(!running()) break;
    if(centreTravel-initialCentre>=target) break;
    keep_center();
    accountMotion();
    if(centreTravel>lastProgress+2) { timeoutStart=millis(); lastProgress=centreTravel; }
    if(millis()-timeoutStart>2000) { fail("FAULT: replay corridor made no progress."); break; }
    // Bound an endlessly oscillating centering routine as well.
    if(wheelTravel-initialTravel > target*4.0f+4000.0f) {
      fail("FAULT: excessive steering during replay."); break;
    }
  }
  stopAndSettle();
}
void replayRoute() {
  for(uint32_t i=0;i<saved.count && running();++i) {
    const Command &c=saved.commands[i];
    if(c.kind==route::CHECK) {
      if(!settleSignature()) break;
      if(stableMask!=c.mask) { fail("FAULT: replay junction differs from saved route; no guessed turn."); break; }
    } else if(c.kind==route::CRUISE) replayCruise(c.left);
    else move(c.speedLeft,c.left,c.speedRight,c.right);
  }
  bool complete=running(); stopAndSettle(); mode=IDLE;
  logMessage(complete?"Replay complete: saved finish distance reached.":"Replay stopped; saved route unchanged.");
}
void startRun(bool replay) {
  if(!hardwareReady) { logMessage("Hardware not ready."); return; }
  if(replay && !savedValid) { logMessage("No valid saved route. Learn with 1, finish with 2."); return; }
  resetMotion(); mode=replay?REPLAY:LEARN;
  if(!settleSignature()) { mode=IDLE; return; }
  if(replay) { logMessage("Replay started."); replayRoute(); return; }
  graph.clear(); recordedCount=edgeFirst=0; currentNode=-1; edgeStartTravel=lastCheckpointTravel=0;
  checkpoint(); logMessage("Learning started. 2 = mark finish/save, 0 = abort without saving.");
}
void help() {
  logMessage("MAZE V6 | Bluetooth: MazeSolver-V6 | 1 learn | 2 finish/save | 3 replay | 0 abort | ? status | C clear (idle)");
  logMessage(savedValid?"Saved route available. Place at SAME start position AND direction before 3.":"No saved route.");
}
void dumpMap() {
  logMessage("RAM map: encoder-count coordinates; heading in degrees. Centre mask has bit 128 set.");
  char line[140];
  for(int i=0;i<graph.nodeCount;++i) {
    const auto &n=graph.nodes[i];
    snprintf(line,sizeof(line),"NODE %d x=%.0f y=%.0f heading=%.1f mask=%u",i,n.pose.x,n.pose.y,n.pose.yaw*180/route::PI_F,n.mask);
    logMessage(line);
  }
  for(int i=0;i<graph.edgeCount;++i) {
    const auto &e=graph.edges[i];
    snprintf(line,sizeof(line),"EDGE %u -> %u effort=%.0f commands=%u turn=%d",e.from,e.to,e.cost,e.count,e.turnQuarters);
    logMessage(line);
  }
}
void setup() {
  Serial.begin(115200);
  for(int pin : {AIN1,AIN2,PWMA,BIN1,BIN2,PWMB,F_Sensor,R_Sensor,L_Sensor}) pinMode(pin,OUTPUT);
  brakeBothMotors();
  pinMode(left_encoder_pin,INPUT_PULLUP);
  pinMode(right_encoder_pin,INPUT); // GPIO34 has no internal pull-up; retain external encoder wiring.
  attachInterrupt(digitalPinToInterrupt(left_encoder_pin),leftEncoderISR,RISING);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin),rightEncoderISR,RISING);
  if(!SerialBT.begin("MazeSolver-V6")) { logMessage("Bluetooth initialization failed."); return; }
  Wire.begin(21,22); Wire.setTimeOut(20);
  for(int pin : {F_Sensor,R_Sensor,L_Sensor}) digitalWrite(pin,LOW);
  delay(10);
  digitalWrite(F_Sensor,HIGH); delay(10);
  bool ok=F_lox.begin(0x30,false,&Wire);
  digitalWrite(R_Sensor,HIGH); delay(10); ok=R_lox.begin(0x31,false,&Wire)&&ok;
  digitalWrite(L_Sensor,HIGH); delay(10); ok=L_lox.begin(0x32,false,&Wire)&&ok;
  if(!ok) { logMessage("Sensor initialization failed; motors remain stopped."); return; }
  ok=F_lox.startRangeContinuous(40);
  ok=R_lox.startRangeContinuous(40)&&ok; ok=L_lox.startRangeContinuous(40)&&ok;
  if(!ok) { logMessage("Continuous sensor startup failed."); return; }
  storageReady=preferences.begin("maze-v6",false);
  if(!storageReady) logMessage("NVS unavailable: cannot save routes.");
  loadSaved(); hardwareReady=true;
  uint32_t start=millis(); while(millis()-start<400) { pollSensors(); delay(1); }
  help();
}
void loop() {
  pollSensors(); accountMotion();
  if(mode==LEARN) {
    serviceCommands();
    if(running()) exploreStep();
    if(stopRequested || faulted) {
      stopAndSettle(); flushCruise();
      if(finishRequested && !faulted) saveFinished();
      else logMessage("Learning aborted; previous saved route unchanged.");
      mode=IDLE;
    }
  } else {
    for(Stream *stream : {static_cast<Stream*>(&Serial),static_cast<Stream*>(&SerialBT)}) {
      if(!stream->available()) continue;
      char c=stream->read();
      if(c=='1') startRun(false);
      else if(c=='3') startRun(true);
      else if(c=='C' || c=='c') {
        if(storageReady && (!preferences.isKey("route") || preferences.remove("route"))) {
          savedValid=false; logMessage("Saved route cleared.");
        } else logMessage("Clear failed.");
      } else if(c=='?') help();
      else if(c=='M' || c=='m') dumpMap();
      else if(c=='2' || c=='0' || c=='s' || c=='S') brakeBothMotors();
      if(mode!=IDLE) break;
    }
  }
  delay(1);
}

// Original V5 left-hand decision thresholds and maneuver counts.
void autonomus() {

  if (autoMode) {

    // Obstacle avoidance
    if (l < too_close_distance) {
      moveBothMotors(-motorForwardSpeed, each_step_encoder_value,
                     motorForwardSpeed, 150);
    } else if (r < too_close_distance) {
      moveBothMotors(motorForwardSpeed, each_step_encoder_value,
                     -motorForwardSpeed, each_step_encoder_value);
    } else if (f < too_close_distance) {
      moveBothMotors(-motorForwardSpeed, each_step_encoder_value,
                     -motorForwardSpeed, each_step_encoder_value);
    } else if (l > distance_threshold / 1.6 && l < distance_threshold &&
               r > distance_threshold) {
      moveBothMotors(-motorForwardSpeed, each_step_encoder_value,
                     motorForwardSpeed, each_step_encoder_value);
    }

    // 1. DEAD END: when all three sides are blocked, rotate 180 degrees
    if (f < distance_threshold / 1.2 && r < distance_threshold &&
        l < distance_threshold) {

      // Rotate 180 degrees
      moveBothMotors(-motorTurnSpeed, turn_encoder_value * 2, motorTurnSpeed,
                     turn_encoder_value * 2); // Rotate 180 degrees

      // GO backward
      moveBothMotors(-motorTurnSpeed, 600, -motorTurnSpeed,
                     600); // Move backward a bit
    }

    // 2. ONLY LEFT WAY FOUND: forward and right are blocked
    else if (l > distance_threshold && f < distance_threshold / 1.2 &&
             r < distance_threshold) {

      // Move forward a bit to align with junction, then rotate 90 degrees to the left
      moveBothMotors(motorTurnSpeed, extraSteps / 3, motorTurnSpeed,
                     extraSteps / 3);

      // Rotate 90 degrees to the left
      moveBothMotors(-motorTurnSpeed, turn_encoder_value, motorTurnSpeed,
                     turn_encoder_value);

      // Move forward into the new branch
      moveBothMotors(motorTurnSpeed, extraSteps, motorTurnSpeed,
                     extraSteps);

      responsivePause(rest);
    }

    // 3. ONLY RIGHT WAY FOUND: forward and left are blocked
    else if (r > distance_threshold && f < distance_threshold / 1.5 &&
             l < distance_threshold) {

      // Move forward a bit to align with junction, then rotate 90 degrees to the right
      moveBothMotors(motorTurnSpeed, extraSteps / 3, motorTurnSpeed,
                     extraSteps / 3);

      // Rotate 90 degrees to the right
      moveBothMotors(motorTurnSpeed, turn_encoder_value, -motorTurnSpeed,
                     turn_encoder_value);

      // Move forward into the new branch
      moveBothMotors(motorTurnSpeed, extraSteps, motorTurnSpeed,
                     extraSteps);

      responsivePause(rest);
    }

    // 4. THREE-WAY / FOUR-WAY JUNCTION: Left, Forward, and Right all open -> CHOOSE LEFT
    else if (l > distance_threshold && f > distance_threshold / 1.5 &&
             r > distance_threshold) {

      // Move forward to align with junction
      moveBothMotors(motorTurnSpeed, extraSteps, motorTurnSpeed, extraSteps);

      responsivePause(rest);

      // Rotate 90 degrees to the left (Left-Hand priority)
      moveBothMotors(-motorTurnSpeed, turn_encoder_value, motorTurnSpeed,
                     turn_encoder_value);

      // Move forward into the left branch
      moveBothMotors(motorTurnSpeed, extraSteps, motorTurnSpeed, extraSteps);
    }

    // 5. T-JUNCTION: forward is blocked, Left and Right open -> CHOOSE LEFT
    else if (f < distance_threshold / 1.5 && l > distance_threshold &&
             r > distance_threshold) {

      // Move backward a bit
      moveBothMotors(-motorTurnSpeed, each_step_encoder_value, -motorTurnSpeed,
                     each_step_encoder_value);

      // Rotate 90 degrees to the left (Left-Hand priority)
      moveBothMotors(-motorTurnSpeed, turn_encoder_value, motorTurnSpeed,
                     turn_encoder_value);
      responsivePause(rest);
    }

    // 6. FORWARD AND LEFT OPEN (Right blocked) -> CHOOSE LEFT
    else if (f > distance_threshold / 1.2 && l > distance_threshold &&
             r < distance_threshold) {

      // Move forward to align with junction
      moveBothMotors(motorTurnSpeed, extraSteps, motorTurnSpeed, extraSteps);

      // Rotate 90 degrees to the left (Left-Hand priority over Forward)
      moveBothMotors(-motorTurnSpeed, turn_encoder_value, motorTurnSpeed,
                     turn_encoder_value);

      // Move forward into the left branch
      moveBothMotors(motorTurnSpeed, extraSteps, motorTurnSpeed, extraSteps);
      responsivePause(rest);
    }

    // 7. FORWARD AND RIGHT OPEN (Left blocked) -> KEEP MOVING FORWARD
    // In Left-Hand rule, between Forward and Right, Forward has priority!
    else if (f > distance_threshold && r > distance_threshold &&
             l < distance_threshold) {
      keep_center();
    }

    // 8. ONLY FORWARD OPEN: walls on both Left & Right (straight corridor)
    else if (f > distance_threshold && r < distance_threshold &&
             l < distance_threshold) {
      keep_center();
    }

    // Fallback: keep centering forward
    else {
      keep_center();
    }
  }
}
