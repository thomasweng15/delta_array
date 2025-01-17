#include "delta_array.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include<math.h>

#define NUM_MOTORS 9
#define MY_ID 0

//################################## Feather MC and ADC Libraries INIT #####################3
Adafruit_MotorShield MC0 = Adafruit_MotorShield(0x62);
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60);
Adafruit_MotorShield MC2 = Adafruit_MotorShield(0x61);
//
Adafruit_DCMotor *MC0_M1 = MC0.getMotor(1);
Adafruit_DCMotor *MC0_M2 = MC0.getMotor(2);
Adafruit_DCMotor *MC0_M3 = MC0.getMotor(3);
Adafruit_DCMotor *MC1_M1 = MC1.getMotor(1);
Adafruit_DCMotor *MC1_M2 = MC1.getMotor(2);
Adafruit_DCMotor *MC1_M3 = MC1.getMotor(3);
Adafruit_DCMotor *MC2_M1 = MC2.getMotor(1);
Adafruit_DCMotor *MC2_M2 = MC2.getMotor(2);
Adafruit_DCMotor *MC2_M3 = MC2.getMotor(3);
//
Adafruit_DCMotor* motors[NUM_MOTORS] = {MC0_M1, MC0_M2, MC0_M3, MC1_M1, MC1_M2, MC1_M3, MC2_M1, MC2_M2, MC2_M3};
//Adafruit_DCMotor* motors[NUM_MOTORS] = {MC1_M4};


Adafruit_ADS1015 ADC2;
Adafruit_ADS1015 ADC1;
Adafruit_ADS1015 ADC0;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC0, &ADC0, &ADC0, &ADC1, &ADC1, &ADC1, &ADC2, &ADC2, &ADC2};
int channels[NUM_MOTORS] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
//int channels[NUM_MOTORS] = {3};

//##################################### GLOBAL VARIABLES ###################################3
const int numChars = 256;
float pi = 3.1415926535;

uint8_t input_cmd[numChars];
bool newData = false;

DeltaMessage message = DeltaMessage_init_zero;
static boolean recvInProgress = false;
static byte ndx = 0;
char startMarker = 0xA6;
char confMarker = '~';
char endMarker = 0xA7;

unsigned long current_arduino_time;
unsigned long last_arduino_time;
float time_elapsed;

float joint_positions[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float new_joint_positions[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float joint_errors[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// We need last_joint_errors to compute PID control value for d. 
float last_joint_errors[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// We need total_joint_errors to compute PID control value for i. 
float total_joint_errors[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int motor_val[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool is_movement_done = false;

//################################# SETUP AND LOOP FUNCTIONS ################################3
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  
  // set all the base dc motor control pins to outputs
  MC0.begin();
  MC1.begin();
  MC2.begin();
  // start all the ADCs
  ADC2.begin(0x4A);
  ADC1.begin(0x49);
  ADC0.begin(0x48);

  ADC2.setGain(GAIN_ONE);
  ADC1.setGain(GAIN_ONE);
  ADC0.setGain(GAIN_ONE);

  // disable dc motors by setting their enable lines to low
  for(int i=0; i<NUM_MOTORS; i++){
    motors[i]->setSpeed(150);
    motors[i]->run(RELEASE);
    delay(10);
  }
  
//  printIPAddr();
  
  readJointPositions();
  //resetJoints();
//  stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  recvWithStartEndMarkers();
  if (newData == true) {
//    printIPAddr();
    if (decodeNanopbData()){
      writeJointPositions();
    }
    newData = false;
    ndx = 0;
  }
  else{
    current_arduino_time = millis();
    time_elapsed = float(current_arduino_time - last_arduino_time)/1000;
    last_arduino_time = current_arduino_time;
  }
  // writeJointPositions();
}
