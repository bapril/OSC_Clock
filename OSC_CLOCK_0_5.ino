/* OSC Clock
 * 
 * Version 0.5
 *
 * Notes:
 * - Use caution when shifting. The current code shifts on request, this can cause offset between 
 *     real and expected position of + or - 18 seconds in extreme cases.
 * - Lower gear ratios may not keep time well, testing required.
 *
 * TODO:
 * -
 */

//
// Includes
//
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include "RTClib.h"
#include <SPI.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include "ESP8266TimerInterrupt.h"
#include "ESP8266_ISR_Timer.h"
#include "config.h"

//
// Tests
// 

#ifndef ESP8266
  #error This code is designed to run on ESP8266 platform! Please check your Tools->Board setting.
#endif

//
// Defines
//

//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 32 // OLED display height, in pixels
//#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

//Set the timer 1 prescaler to 1:1, short timer, but more accurate. 
#define USING_TIM_DIV1                true
#define USING_TIM_DIV16               false
#define USING_TIM_DIV256              false

#define STARTUP_GEAR  1
#define GEAR_MASK_1   B00011111
#define GEAR_MASK_2   B00001111
#define GEAR_MASK_4   B00000111
#define GEAR_MASK_8   B00000011
#define GEAR_MASK_16  B00000001
#define GEAR_MASK_32  B00000000
#define GEAR_STEP_1   18000
#define GEAR_STEP_2   9000
#define GEAR_STEP_4   4500
#define GEAR_STEP_8   2250
#define GEAR_STEP_16  1125
#define GEAR_STEP_32  562.5

#define MODE_STANDBY  0
#define MODE_RUN      1
#define MODE_MOVE     2
#define MODE_STOP     3

#define OPTIC_PIN     D7
#define MOT_STEP      D0
#define MOT_DIR       D3
#define MOT_EN        D4
#define MOT_M0        D5
#define MOT_M1        D6
#define MOT_M2        D8

#define IN_STEP       2
#define INTER_STEP    2
#define MIN_SYNC_STEPS 2 //minimum number of steps for a run_mode correction? 

//
// Global Variables
//

int status = WL_IDLE_STATUS;     // the Wifi radio's status

bool start_step = false; //Need a step
bool homed = false;
bool dir = true; //true = cloclwise. 
bool shift_requested = false;
bool sync_requested = false;

int in_step = 0; // How many ticks before ending the step. 
int to_step = 0; // how many ticks befor the next step;
int steps_to_go = 0;
int sys_mode = MODE_STANDBY;
int tick_count = 0; //Count the inner_step loop (32 steps per step)
int gear;
int gear_mask;
int gear_milis;
int face_milis = 0;
int odo = 0;
int next_mode = MODE_STOP;

//
//Initializers
//

RTC_DS3231 rtc;
WiFiUDP Udp;
ESP8266Timer ITimer;
ESP8266_ISR_Timer ISR_Timer;
extern ESP8266_ISR_Timer ISR_Timer;  // declaration of the global variable ISRTimer
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//
// Declare ISR Functions
//

void IRAM_ATTR isr_tick() {
  ISR_Timer.run();
}

void IRAM_ATTR TimerHandler() {
  int c;
  tick_count++;
  if (tick_count == 32){
    tick_count = 0;
    start_step = true;
    sync_requested = true;
  } else {
    c = tick_count & gear_mask;
    if (c == 0){
      start_step = true;
    }
  }
}

void IRAM_ATTR isr_step() {
  if (in_step > 0){
    in_step --;
    if (!in_step) {
      digitalWrite(MOT_STEP,LOW);
      if (sys_mode == MODE_MOVE){
        steps_to_go--;
        if (!steps_to_go){
          Serial.println("Move Done");
          set_next_mode();
        } else {
          to_step = INTER_STEP;
        }
      }
    }
  } else {
    switch (sys_mode) {
      case MODE_MOVE:
        if(to_step){
          to_step--;
          if (!to_step)
            begin_step(IN_STEP);
        }      
        break;
       case MODE_RUN:
         if (start_step){
           start_step = false;
           begin_step(IN_STEP);
           steps_to_go = 2; // Trick step-down so it doesn't take us out of move. 
         }
         break;
    }
  }
}

void IRAM_ATTR home_detected() {
  if (!homed){
    Serial.println("HOME");
    mode_stop();
    homed = true;
  }
  face_milis = 0;
}

//
// Declare OSC Functions
//

/*
 * //rtc/set $year $month $day $hour $minute $second
 * 
 * /clock/home - Move forward to midnight.
 * /clock/stop - Stop the Clock
 * /clock/now - Move to current time.
 * /clock/run - Move @ 1:1
 * /clock/odo - read then reset Odometer. 
 * /clock/gear $gear - Set the clock gear.
 * 
 * //clock/move/speed $speed - Set Movement speed 
 * //clock/move/cw/to $hour $minute $second  - Run forward to $time at $speed
 * //clock/move/ccw/to $hour $minute $second - Run backwards to $time at $speed
 */

void clock_home(OSCMessage &msg, int addrOffset){
  homed = false;
  mode_move(50000,true,MODE_STOP); 
}

void clock_stop(OSCMessage &msg, int addrOffset){
  mode_stop();
}

void clock_now(OSCMessage &msg, int addrOffset){
  sync_clock(MODE_STOP);
}

void clock_run(OSCMessage &msg, int addrOffset){
  mode_run();
}

void clock_odo(OSCMessage &msg, int addrOffset){
  Serial.print("\n ODO: ");
  Serial.println(odo);
  odo = 0;
}

void clock_gear(OSCMessage &msg, int addrOffset){
  if (msg.isInt(0)) {
    req_shift(msg.getInt(0));
  } else {
    Serial.println("OSC variable 0 not of type INT");
  }
}

void ClockAction(OSCMessage &msg, int addrOffset){
  msg.route("/home",clock_home, addrOffset);
  msg.route("/stop", clock_stop, addrOffset);
  msg.route("/now", clock_now, addrOffset);
  msg.route("/run", clock_run, addrOffset);  
  msg.route("/odo", clock_odo, addrOffset);
  msg.route("/gear", clock_gear, addrOffset);
//  msg.route("/speed", clock_speed, addrOffset);
//  msg.route("/enable", enableStepper, addrOffset);
//  msg.route("/disable", disableStepper, addrOffset);
}

void RTCAction(OSCMessage &msg, int addrOffset){
  // msg.route("/set" RTC_Set, addrOffset);
  // msg.route("/get" RTC_Get, addrOffset);
}

void OSCMsgReceive() {
  OSCMessage msgIN;
  char address[255];
  int size;
  if((size = Udp.parsePacket())>0) {
    while(size--)
      msgIN.fill(Udp.read());
    if(!msgIN.hasError())
      msgIN.getAddress(address, 0);
      Serial.println(address);
      msgIN.route("/clock", ClockAction);
      //msgIN.route("/rtc", RTCAction);
  }
}

//
// Declare Utility Functions
//

void display_time(){
  DateTime now = rtc.now();
  Serial.println(" "+now.timestamp(DateTime::TIMESTAMP_FULL));
}

void motor_enable(){
  digitalWrite(MOT_EN, LOW);
}
void motor_disable(){
  digitalWrite(MOT_EN, HIGH);
}

void dir_cw(){
  dir = true;
  digitalWrite(MOT_DIR, HIGH);
}

void dir_ccw(){
  dir = false;
  digitalWrite(MOT_DIR, LOW);
}

void mode_standby(){
  sys_mode = MODE_STANDBY;
  motor_disable();
}
void mode_run(){
  dir_cw();
  motor_enable();
  sys_mode = MODE_RUN;
}

void begin_step(int step_time){
  in_step = step_time;
  digitalWrite(MOT_STEP,HIGH);
  if(dir){
    face_milis += gear_milis;
  } else {
    face_milis -= gear_milis;
  }
  odo++;
}

void mode_move(unsigned long steps,bool clockwise, int after){
  if (steps < 1)
    return;
  next_mode = after;
  if(clockwise){
    dir_cw();
  } else {
    dir_ccw();
  }
  motor_enable();
  steps_to_go = steps;
  sys_mode = MODE_MOVE;
  to_step = 1;
}

void mode_stop(){
  sys_mode = MODE_STOP;
  motor_disable();
}

unsigned long get_rtc_face_millis(){
  DateTime now = rtc.now();
  int hour = now.hour();
  if (hour > 12) 
    hour = hour - 12;
  return (now.second() + (now.minute() * 60) + (hour * 3600)) * 1000;
}

void shift(int req_gear){
  //TODO if we are not enable, don't re-enable at the end.
  gear = req_gear;
  motor_enable();
  switch (req_gear) {
  case 1:
    Serial.println("Gear 1");
    gear_mask = GEAR_MASK_1;
    gear_milis = GEAR_STEP_1;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,LOW);
    break;
  case 2:
    Serial.println("Gear 2");
    gear_mask = GEAR_MASK_2;
    gear_milis = GEAR_STEP_2;
    digitalWrite(MOT_M0,HIGH);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,LOW);
    break;
  case 4:
    Serial.println("Gear 4");
    gear_mask = GEAR_MASK_4;
    gear_milis = GEAR_STEP_4;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,HIGH);
    digitalWrite(MOT_M2,LOW);
    break;
  case 8:
    Serial.println("Gear 8");
    gear_mask = GEAR_MASK_8;
    gear_milis = GEAR_STEP_8;
    digitalWrite(MOT_M0,HIGH);
    digitalWrite(MOT_M1,HIGH);
    digitalWrite(MOT_M2,LOW);
    break;
  case 16:
    Serial.println("Gear 16");
    gear_mask = GEAR_MASK_16;
    gear_milis = GEAR_STEP_16;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,HIGH);
    break;
  case 32:
    Serial.println("Gear 32");
    gear_mask = GEAR_MASK_32;
    gear_milis = GEAR_STEP_32;
    digitalWrite(MOT_M0,HIGH);
    digitalWrite(MOT_M1,HIGH);
    digitalWrite(MOT_M2,HIGH);
    break;
  }
  motor_enable();
  shift_requested = false;
}

void req_shift(int req_gear){
  if (req_gear == gear)
    return;
  if (!(req_gear == 1 || req_gear == 2 || req_gear == 4 || req_gear == 8 || req_gear == 16 || req_gear == 32)){
    Serial.print("Invalid gear selected: ");
    Serial.println(req_gear);
    return;
  }
  shift_requested = true;
  shift(req_gear);
}

void sync_clock(int after){
  bool cw;
  unsigned long target_face_milli = get_rtc_face_millis();
  unsigned long steps_to_go = 0;
  unsigned long delta_milis = 0;
  dir_cw();
  //TODO choose shortest direction to move.
  if(target_face_milli > face_milis){
    delta_milis = (target_face_milli - face_milis);
    cw = true;
  } else {
    Serial.println("====== Sync_reverse ======");
    delta_milis = (face_milis - target_face_milli);
    cw = false;
  }
  if(delta_milis > gear_milis * MIN_SYNC_STEPS){
    steps_to_go = (int)delta_milis/gear_milis;
    Serial.print("Target Milis: ");
    Serial.println(target_face_milli);
    Serial.print("Face Milis: ");
    Serial.println(face_milis);
    Serial.print("Delta Milis:");
    Serial.println(delta_milis);
    Serial.print("Steps: ");
    Serial.println(steps_to_go);
    mode_move(steps_to_go,cw,after); 
  } else {
    Serial.print("Delta Milis:");
    Serial.println(delta_milis);
  }
}

void set_next_mode(){
  switch (next_mode){
    case MODE_STOP:
      mode_stop();
      break;
    case MODE_RUN:
      mode_run();
      break;
    default:
      mode_stop();
  }
}

//
// Declare Arduino Functions
//

void setup() {
  req_shift(STARTUP_GEAR);
  mode_standby();

  ESP.wdtEnable(1000);
  Wire.begin();
  
  Serial.begin(115200);  // start serial for output
  while (!Serial);
  Serial.println("\nBegin:");

  //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //  Serial.println(F("SSD1306 allocation failed"));
  //  for(;;); // Don't proceed, loop forever
  //}
  
  pinMode(OPTIC_PIN, INPUT); //optical sensor.
  pinMode(MOT_EN, OUTPUT);
  pinMode(MOT_DIR, OUTPUT);
  pinMode(MOT_STEP, OUTPUT);
  pinMode(MOT_M0, OUTPUT);
  pinMode(MOT_M1, OUTPUT);
  pinMode(MOT_M2, OUTPUT);
  
  digitalWrite(MOT_M0, LOW);
  digitalWrite(MOT_M1, LOW);
  digitalWrite(MOT_M2, LOW);

  attachInterrupt(digitalPinToInterrupt(OPTIC_PIN), home_detected, RISING);


  Serial.println("Pins Set");
  
  if (! rtc.begin(&Wire)) {
    Serial.println("RTC: Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  } else {
    Serial.print("RTC: enabled:");
    display_time(); 
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    ESP.wdtFeed();
    Serial.print(".");
  }

  Serial.println(WiFi.localIP());
  Udp.begin(inPort);
  Serial.println("OSC Started");
  
  Serial.println("Timers Started");
  Serial.print("RTC Time: ");
  display_time(); 
  unsigned long target_face_milli = get_rtc_face_millis();
  Serial.print("Target_Face_Milli:");
  Serial.println(target_face_milli);

  if (ITimer.attachInterruptInterval(20, isr_tick)) {
    Serial.print(F("Starting  ITimer OK, millis() = ")); Serial.println(millis());
  } else {
    Serial.println(F("Can't set ITimer correctly. Select another freq. or interval"));
  } 
  ISR_Timer.setInterval(1, isr_step);
  ISR_Timer.setInterval(562.5, TimerHandler);

  mode_move(50000,true,MODE_STOP);

  Serial.println("Homing");
}

void loop(){ 
  ESP.wdtFeed();
  OSCMsgReceive();
  yield();
  if(sync_requested){
    sync_requested = false;
    if(sys_mode == MODE_RUN){
       sync_clock(MODE_RUN);
    }
  }
}
