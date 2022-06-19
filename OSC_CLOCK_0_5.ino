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

#define D_MODE_X      0
#define D_MODE_Y      0
#define D_GEAR_X      64
#define D_RTC_X       0
#define D_RTC_Y       8
#define D_FACE_X      0
#define D_FACE_Y      16
#define D_OSC_X       0
#define D_OSC_Y       24

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
bool update_display = false;

uint8_t sys_mode = MODE_STANDBY;
uint8_t gear;
uint8_t next_mode = MODE_STOP;
uint8_t display_refresh_state = 0;

int display_refresh_counter = 0;
int tick_count = 0; //Count the inner_step loop (32 steps per step)
int gear_mask;
int steps_to_go = 0;
int in_step = 0; // How many ticks before ending the step.
int to_step = 0; // how many ticks befor the next step;

DateTime rtc_now;

unsigned long face_milis = 0;
unsigned long gear_milis;

String last_OSC_command = "NO-CMD";

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

void clock_home(__attribute__((unused)) OSCMessage &msg, __attribute__((unused)) int addrOffset){
  homed = false;
  mode_move(50000,true,MODE_STOP); 
}

void clock_stop(__attribute__((unused)) OSCMessage &msg, __attribute__((unused)) int addrOffset){
  mode_stop();
}

void clock_now(__attribute__((unused)) OSCMessage &msg, __attribute__((unused)) int addrOffset){
  sync_clock(MODE_STOP);
}

void clock_run(__attribute__((unused)) OSCMessage &msg, __attribute__((unused)) int addrOffset){
  mode_run();
}

void clock_gear(OSCMessage &msg, __attribute__((unused)) int addrOffset){
  if (msg.isInt(0)) {
    req_shift(msg.getInt(0));
  }
}

void ClockAction(OSCMessage &msg, __attribute__((unused)) int addrOffset){
  msg.route("/home",clock_home, addrOffset);
  msg.route("/stop", clock_stop, addrOffset);
  msg.route("/now", clock_now, addrOffset);
  msg.route("/run", clock_run, addrOffset);  
  msg.route("/gear", clock_gear, addrOffset);
//  msg.route("/speed", clock_speed, addrOffset);
//  msg.route("/enable", enableStepper, addrOffset);
//  msg.route("/disable", disableStepper, addrOffset);
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
      last_OSC_command = String(address);
      update_display = true;
      msgIN.route("/clock", ClockAction);
    }
  }
}

//
// Declare Utility Functions
//

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
  setMode(MODE_STANDBY);
  motor_disable();
}

void mode_run(){
  dir_cw();
  motor_enable();
  setMode(MODE_RUN);
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
  setMode(MODE_MOVE);
  to_step = 1;
}

void mode_stop(){
  setMode(MODE_STOP);
  motor_disable();
}

unsigned long get_rtc_face_millis(){
  int hour = rtc_now.hour();
  if (hour > 12) 
    hour = hour - 12;
  return (rtc_now.second() + (rtc_now.minute() * 60) + (hour * 3600)) * 1000;
}

void shift(int req_gear){
  //TODO if we are not enable, don't re-enable at the end.
  gear = req_gear;
  update_display = true;
  motor_disable();
  switch (req_gear) {
  case 1:
    gear_mask = GEAR_MASK_1;
    gear_milis = GEAR_STEP_1;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,LOW);
    break;
  case 2:
    gear_mask = GEAR_MASK_2;
    gear_milis = GEAR_STEP_2;
    digitalWrite(MOT_M0,HIGH);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,LOW);
    break;
  case 4:
    gear_mask = GEAR_MASK_4;
    gear_milis = GEAR_STEP_4;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,HIGH);
    digitalWrite(MOT_M2,LOW);
    break;
  case 8:
    gear_mask = GEAR_MASK_8;
    gear_milis = GEAR_STEP_8;
    digitalWrite(MOT_M0,HIGH);
    digitalWrite(MOT_M1,HIGH);
    digitalWrite(MOT_M2,LOW);
    break;
  case 16:
    gear_mask = GEAR_MASK_16;
    gear_milis = GEAR_STEP_16;
    digitalWrite(MOT_M0,LOW);
    digitalWrite(MOT_M1,LOW);
    digitalWrite(MOT_M2,HIGH);
    break;
  case 32:
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
    delta_milis = (face_milis - target_face_milli);
    cw = false;
  }
  if(delta_milis > gear_milis * MIN_SYNC_STEPS){
    steps_to_go = (int)delta_milis/gear_milis;
    mode_move(steps_to_go,cw,after); 
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

String IpAddress2String(const IPAddress& ipAddress){
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}

void setMode(int mode){
  sys_mode = mode;
  update_display = true;
}

//
// Display Funtions
//

void display_timestamp(){
  #ifdef DIS_SERIAL
  Serial.println(rtc_now.timestamp(DateTime::TIMESTAMP_FULL));
  #endif
  #ifdef DIS_SSD1306
  display.println(rtc_now.timestamp(DateTime::TIMESTAMP_FULL));
  display.display();
  #endif
}

void display_time(){
  #ifdef DIS_SERIAL
  Serial.println(rtc_now.timestamp(DateTime::TIMESTAMP_TIME));
  #endif
  #ifdef DIS_SSD1306
  display.println(rtc_now.timestamp(DateTime::TIMESTAMP_TIME));
  display.display();
  #endif
}

void display_face_time(){
  String face_time = "";
  int hour, minute, second;
  int my_face_mil = face_milis;
  hour = int(face_milis / 3600000);
  my_face_mil = face_milis - (hour * 3600000);
  minute = int(my_face_mil / 60000);
  my_face_mil = my_face_mil - (minute * 60000);
  second = int(my_face_mil / 1000);

  face_time = String(hour)+":"+String(minute)+":"+String(second);
#ifdef DIS_SERIAL
  Serial.println(face_time);
#endif
#ifdef DIS_SSD1306
  display.println(face_time);
  display.display();
#endif
}

void sayln(String input){
  if(!display_ready){ return;} //May not need anymore now called in displayGear
#ifdef DIS_SERIAL
  Serial.println(input);
#endif
#ifdef DIS_SSD1306
  display.println(input);
  display.display();
#endif
}

void say(String input){
  if(!display_ready){ return;} //May not need anymore now called in displayGear
#ifdef DIS_SERIAL
  Serial.print(input);
#endif
#ifdef DIS_SSD1306
  display.print(input);
  display.display();
#endif
}

void displayMode(){
  if(!display_ready){ return;} //May not need anymore now called in displayGear
#ifdef DIS_SSD1306
  display.fillRect(D_MODE_X, D_MODE_Y, D_GEAR_X, 8, SSD1306_BLACK);
  display.setCursor(D_MODE_X,D_MODE_Y);
#endif
  switch(sys_mode){
  case MODE_STANDBY:
    say("M:STANDBY");
    break;
  case MODE_RUN:
    say("M:RUN    ");
    break;
  case MODE_MOVE:
    say("M:MOVE   ");
    break;
  case MODE_STOP:
    say("M:STOP   ");
    break;
  }
#ifdef DIS_SERIAL
  Serial.println("");
#endif
}

void displayGear(){
  if(!display_ready){ return;} //May not need anymore now called in displayGear
#ifdef DIS_SSD1306
  display.fillRect(D_GEAR_X, D_MODE_Y, D_GEAR_X, 8, SSD1306_BLACK);
  display.setCursor(D_GEAR_X,D_MODE_Y);
#endif
  say("G: "+String(gear));
#ifdef DIS_SERIAL
  Serial.println("");
#endif
}

void displayRTCTime(){
#ifdef DIS_SSD1306
  display.fillRect(D_RTC_X, D_RTC_Y, 128, 8, SSD1306_BLACK);
  display.setCursor(D_RTC_X,D_RTC_Y);
#endif
  say("RTC: ");
  display_time();
  display.display();
}

void displayFaceTime(){
#ifdef DIS_SSD1306
  display.fillRect(D_FACE_X, D_FACE_Y, 128, 8, SSD1306_BLACK);
  display.setCursor(D_FACE_X,D_FACE_Y);
#endif
  say("FFace: ");
  display_face_time();
}

void displayOSC(){
#ifdef DIS_SSD1306
  display.fillRect(D_OSC_X, D_OSC_Y, 128, 8, SSD1306_BLACK);
  display.setCursor(D_OSC_X,D_OSC_Y);
#endif
  say("OSC: "+String(last_OSC_command));
#ifdef DIS_SERIAL
  Serial.println("");
#endif
}

void clear_display(){
#ifdef DIS_SSD1306
  display.clearDisplay();
  display.setCursor(0,0);
  display.display();
#endif
}

void displayRefresh(){
  switch(display_refresh_state){
  case 0:
    displayMode();
    break;
  case 1:
    displayGear();
    break;
  case 2:
    rtc_now = rtc.now();
    displayRTCTime();
    break;
  case 3:
    displayFaceTime();
    break;
  case 4:
    displayOSC();
    break;
  }
  display_refresh_state++;
  if(display_refresh_state > 4){ display_refresh_state = 0;}
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
    rtc_now = rtc.now();
    display_timestamp();
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

#ifdef DIS_SSD1306
  display.invertDisplay(false);
#endif

  if (ITimer.attachInterruptInterval(20, isr_tick)) {
    say("Starting  ITimer");
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
    delay(100);
    digitalWrite(MOT_STEP,LOW);
    delay(100);
  }
  motor_disable();
  dir_cw();
  mode_move(50000,true,MODE_STOP);
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
  display_refresh_counter ++;
  if(display_refresh_counter > 2048){
    displayRefresh();
    update_display = false;
  }
}
