#include <Math.h>
#include <ArduinoJson.h>
#include "ESPRotary.h"
#include <Pushbutton.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BAUDRATE 115200

///////////////////// - ENCODERS
#define orientationEncoder_pin_clk 2
#define orientationEncoder_pin_dt 4

#define propulsionEncoder_pin_clk 12
#define propulsionEncoder_pin_dt 14
#define propulsionEncoder_pin_sw 27

#define orientationEncoder_start_pos 0
#define orientationEncoder_increment_step 5 
#define orientationEncoder_clicks_per_steps 4
#define orientationEncoder_speed_up_steps 10

#define propulsionEncoder_start_pos 0
#define propulsionEncoder_increment_step 5
#define propulsionEncoder_clicks_per_steps 4 
#define propulsionEncoder_speed_up_steps 10

/////////////////////////////////////////////////////////////////

ESPRotary orientationEncoder;
ESPRotary propulsionEncoder;
Pushbutton direction_button ( propulsionEncoder_pin_sw );
Adafruit_BNO055 bno = Adafruit_BNO055();

double direction_button_press_start = 0;
double direction_button_backward_threshold = 3000;

int lastStepOrientation = 0;
int delta_stepsOrientation = 0;

int lastStepPropulsion = 0;
int delta_stepsPropulsion = 0;

bool button_has_been_pressed = false;

float pitch = 0;
float roll = 0;
float yaw = 0;
float yaw_previous = 0; 
float yaw_delta = 0; 


unsigned long tick = 0;
unsigned long data_send_delay = 500;
float data_send_timer = data_send_delay;

/////////////////////////////////////////////////////////////////

//======================= ardujson
const byte numChars = 64;//buffer length
const int capacity_json = 128;

StaticJsonDocument<capacity_json> msg_from_companion;
StaticJsonDocument<capacity_json> msg_to_companion;

void setup() {

  Serial.begin(BAUDRATE);

  msg_to_companion["pitch"] = 0;
  msg_to_companion["roll"] = 0;
  msg_to_companion["yaw"] = 0;
  msg_to_companion["delta_yaw"] = 0;
  msg_to_companion["propulsion"] = 0;
  msg_to_companion["longPress"] = false;
  msg_to_companion["shortPress"] = false;

  enableOrientationEncoder();
  enablePropulsionEncoder();

  enableBNU();

}


void enableBNU(){
    // Initialisation du capteur BNO055
  if (!bno.begin()) {
    Serial.println("Le capteur BNO055 n'a pas été détecté.");
    while (1);
  }

  Serial.println("Bnu initialized");
}

float delta_time(){

  unsigned long current_tick = millis();
  unsigned long dt = current_tick - tick;

  tick = current_tick;

  return float(dt); 

}



void enableOrientationEncoder(){

  orientationEncoder.begin(
    orientationEncoder_pin_clk, 
    orientationEncoder_pin_dt, 
    orientationEncoder_clicks_per_steps 
  );
  

  orientationEncoder.setLeftRotationHandler(onCWOrientation);
  orientationEncoder.setRightRotationHandler(onCCWOrientation);

  orientationEncoder.retriggerEvent(false);
  orientationEncoder.enableSpeedup(true);
  orientationEncoder.setSpeedupIncrement( orientationEncoder_speed_up_steps );

}


void enablePropulsionEncoder(){

  propulsionEncoder.begin(
    propulsionEncoder_pin_clk, 
    propulsionEncoder_pin_dt, 
    propulsionEncoder_clicks_per_steps
  );

  propulsionEncoder.setLeftRotationHandler( onCWPropulsion );
  propulsionEncoder.setRightRotationHandler( onCCWPropulsion );

  propulsionEncoder.retriggerEvent(false);
  propulsionEncoder.enableSpeedup(true);
  propulsionEncoder.setSpeedupIncrement( propulsionEncoder_speed_up_steps );

}


void onCWOrientation(ESPRotary& r) {

  int increment = r.getPosition();

  delta_stepsOrientation = abs( increment - lastStepOrientation );
  lastStepOrientation = increment;

  msg_to_companion["orientation"] = delta_stepsOrientation;

  update_companion( );

  msg_to_companion["orientation"] = 0;

}


void onCCWOrientation(ESPRotary& r) {

  int increment = r.getPosition();
  delta_stepsOrientation = -abs( increment - lastStepOrientation );
  lastStepOrientation = increment;

  msg_to_companion["orientation"] = delta_stepsOrientation;

  update_companion( );

  msg_to_companion["orientation"] = 0;

}


void onCWPropulsion(ESPRotary& r) {

  int increment = r.getPosition();

  delta_stepsPropulsion = abs( increment - lastStepOrientation );
  lastStepPropulsion = increment;

  msg_to_companion["propulsion"] = delta_stepsPropulsion;

  update_companion( );

  msg_to_companion["propulsion"] = 0;

}


void onCCWPropulsion(ESPRotary& r) {

  int increment = r.getPosition();

  delta_stepsPropulsion = -abs( increment - lastStepOrientation );
  lastStepPropulsion = increment;

  msg_to_companion["propulsion"] = delta_stepsPropulsion;

  update_companion( );

  msg_to_companion["propulsion"] = 0;
  
}


void read_encoders(){

  orientationEncoder.loop();
  propulsionEncoder.loop();

}

void read_button( ){

  if ( direction_button.getSingleDebouncedRelease() )
  {
    
    if ( msg_to_companion["longPress"] == false ) msg_to_companion["shortPress"] = true;

    update_companion( );

    msg_to_companion["longPress"] = false;
    msg_to_companion["shortPress"] = false;

    button_has_been_pressed = false;

  }

  if ( direction_button.getSingleDebouncedPress() )
  {
    direction_button_press_start = millis();
    button_has_been_pressed = true;
  }

  if( direction_button.isPressed() ){
      
    unsigned long current_tick = millis();
    unsigned long dt = current_tick - direction_button_press_start;

    if( dt > direction_button_backward_threshold && button_has_been_pressed == true ) msg_to_companion["longPress"] = true;

  }

}


void readIMU(){

  data_send_timer -= delta_time(); 

  sensors_event_t event;
  bno.getEvent(&event);

  pitch = event.orientation.x;
  roll = event.orientation.y;
  yaw = event.orientation.z;

  yaw_delta = yaw - yaw_previous; 

  if( data_send_timer < 0 ){

    data_send_timer = float(data_send_delay);
    
    msg_to_companion["pitch"] = pitch;
    msg_to_companion["roll"] = roll;
    msg_to_companion["yaw"] = yaw;
    msg_to_companion["delta_yaw"] = yaw_delta;

    update_companion();
    
  }

}



void update_companion( ){

  serializeJson( msg_to_companion, Serial );
  Serial.println();
    
}

void loop() {

  read_encoders( );
  read_button( );
  readIMU( );

  delta_stepsOrientation = 0; 

  delay(10);

}

/////////////////////////////////////////////////////////////////