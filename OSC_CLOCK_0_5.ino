/* OSC Clock
 * 
 * Version 0.5
 *
 * Notes:
 * - Use caution when shifting. The current code shifts on request, this can cause offset between 
 *     real and expected position of + or - 18 seconds in extreme cases.
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

#include "ESP8266TimerInterrupt.h"
#include "ESP8266_ISR_Timer.h"
#include "config.h"

#ifdef DIS_SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

//
// Tests
// 

#ifndef ESP8266
  #error This code is designed to run on ESP8266 platform! Please check your Tools->Board setting.
#endif

//
// Defines
//

#ifdef DIS_SSD1306
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#endif

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
bool display_ready = false;

int in_step = 0; // How many ticks before ending the step. 
int to_step = 0; // how many ticks befor the next step;
int steps_to_go = 0;
int sys_mode = MODE_STANDBY;
int tick_count = 0; //Count the inner_step loop (32 steps per step)
int gear;
int gear_mask;
int gear_milis;
int next_mode = MODE_STOP;

unsigned long face_milis = 0;

//
//Initializers
//

RTC_DS3231 rtc;
WiFiUDP Udp;
ESP8266Timer ITimer;
ESP8266_ISR_Timer ISR_Timer;
extern ESP8266_ISR_Timer ISR_Timer;  // declaration of the global variable ISRTimer

#ifdef DIS_SSD1306
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

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

void clock_gear(OSCMessage &msg, int addrOffset){
  if (msg.isInt(0)) {
    req_shift(msg.getInt(0));
  }
}

void ClockAction(OSCMessage &msg, int addrOffset){
  msg.route("/home",clock_home, addrOffset);
  msg.route("/stop", clock_stop, addrOffset);
  msg.route("/now", clock_now, addrOffset);
  msg.route("/run", clock_run, addrOffset);  
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
    if(!msgIN.hasError()) {
      msgIN.getAddress(address, 0);
      sayln("OSC:"+String(address));
      msgIN.route("/clock", ClockAction);
      //msgIN.route("/rtc", RTCAction);
    }
  }
}

//
// Declare Utility Functions
//

void display_time(){
  DateTime now = rtc.now();
  #ifdef DIS_SERIAL
  Serial.println(" "+now.timestamp(DateTime::TIMESTAMP_FULL));
  #endif
  #ifdef DIS_SSD1306
  display.println(" "+now.timestamp(DateTime::TIMESTAMP_FULL));
  display.display();
  #endif
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
    sayln("Gear 1");
    gear_mask = GEAR_MASK_1;
    gear_milis = GEAR_STEP_1;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,LOW);
    break;
  case 2:
    sayln("Gear 2");
    gear_mask = GEAR_MASK_2;
    gear_milis = GEAR_STEP_2;
    digitalWrite(MOT_M0,HIGH);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,LOW);
    break;
  case 4:
    sayln("Gear 4");
    gear_mask = GEAR_MASK_4;
    gear_milis = GEAR_STEP_4;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,HIGH);
    digitalWrite(MOT_M2,LOW);
    break;
  case 8:
    sayln("Gear 8");
    gear_mask = GEAR_MASK_8;
    gear_milis = GEAR_STEP_8;
    digitalWrite(MOT_M0,HIGH);
    digitalWrite(MOT_M1,HIGH);
    digitalWrite(MOT_M2,LOW);
    break;
  case 16:
    sayln("Gear 16");
    gear_mask = GEAR_MASK_16;
    gear_milis = GEAR_STEP_16;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,HIGH);
    break;
  case 32:
    sayln("Gear 32");
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
    say("Invalid gear selected: ");
    sayln(String(req_gear));
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
    sayln("====== Sync_reverse ======");
    delta_milis = (face_milis - target_face_milli);
    cw = false;
  }
  if(delta_milis > gear_milis * MIN_SYNC_STEPS){
    steps_to_go = (int)delta_milis/gear_milis;
/*    Serial.print("Target Milis: ");
    Serial.println(target_face_milli);
    Serial.print("Face Milis: ");
    Serial.println(face_milis);
    Serial.print("Delta Milis:");
    Serial.println(delta_milis);
    Serial.print("Steps: ");
    Serial.println(steps_to_go); */
    mode_move(steps_to_go,cw,after); 
  } else {
    say("Delta Milis:");
    sayln(String(delta_milis));
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

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}

void sayln(String input){
  if(!display_ready){ return;}
#ifdef DIS_SERIAL
  Serial.println(input);
#endif
#ifdef DIS_SSD1306
  display.println(input);
  display.display();
#endif
}

void say(String input){
  if(!display_ready){ return;}
#ifdef DIS_SERIAL
  Serial.print(input);
#endif
#ifdef DIS_SSD1306
  display.print(input);
  display.display();
#endif
}

void clear_display(){
#ifdef DIS_SSD1306
  display.clearDisplay();
  display.setCursor(0,0);
  display.display();
#endif
}

//
// Declare Arduino Functions
//

void setup() {
  req_shift(STARTUP_GEAR);
  mode_standby();

  ESP.wdtEnable(1000);
  Wire.begin();

#ifdef DIS_SERIAL
  Serial.begin(115200);  // start serial for output
  while (!Serial);
  Serial.println("\nBegin:");
#endif

#ifdef DIS_SSD1306
  //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.invertDisplay(true);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.cp437(true);
#endif
  display_ready = true;

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

  sayln("Pins Set");
  
  if (! rtc.begin(&Wire)) {
    sayln("RTC: Couldn't find RTC");
    while (1) delay(10);
  } else {
    sayln("RTC: enabled:");
    display_time(); 
  }

  if (rtc.lostPower()) {
    sayln("RTC lost power, let's set the time!");
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
    say(".");
  }
  sayln("");
  say("Wifi: ");
  sayln(IpAddress2String(WiFi.localIP()));
  Udp.begin(inPort);
  clear_display();
  sayln("OSC Started");

  unsigned long target_face_milli = get_rtc_face_millis();
  sayln("Target_Face_Milli:"+String(target_face_milli));

  if (ITimer.attachInterruptInterval(20, isr_tick)) {
    say(F("Starting  ITimer OK, millis() = ")); Serial.println(millis());
  } else {
    sayln(F("Can't set ITimer correctly. Select another freq. or interval"));
  } 
  ISR_Timer.setInterval(1, isr_step);
  ISR_Timer.setInterval(562.5, TimerHandler);

  //back-step so that if we at home, we find it again.
  dir_ccw();
  motor_enable();
  for(int i = 0; i < 5; i++){
    digitalWrite(MOT_STEP,HIGH);
    delay(1);
    digitalWrite(MOT_STEP,LOW);
    delay(1);
  }
  motor_disable();
  dir_cw();
  mode_move(50000,true,MODE_STOP);
  
#ifdef DIS_SSD1306
  display.invertDisplay(false);
  display.display();
#endif
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
