
float p = 300.0;
float i_pid = 0.1;
float d = 3.75;

float position_threshold = 0.00035;
//############################# READ / WRITE JOINT POSITIONS #######################################3
void readJointPositions(){
  for(int i = 0; i < NUM_MOTORS; i++){
    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]); 
    joint_positions[i] = motor_val[i] * 0.00006; // 100mm / 1650
  }
}

void writeJointPositions(){
  bool reached_point = false;
  last_arduino_time = millis();
  while (!reached_point){
    current_arduino_time = millis();
    time_elapsed = float(current_arduino_time - last_arduino_time) / 1000.0;
    readJointPositions();
    reached_point = true;
    for(int i = 0; i < NUM_MOTORS; i++){
      joint_errors[i] = joint_positions[i] - new_joint_positions[i];
      
      float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
      
      if(joint_errors[i] > position_threshold){
        int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0);
        reached_point = reached_point && false;
        motors[i]->setSpeed(motor_speed);
        motors[i]->run(BACKWARD);
        total_joint_errors[i] += joint_errors[i];
      }
      else if(joint_errors[i] < -position_threshold){
        int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
        reached_point = reached_point && false;
        motors[i]->setSpeed(motor_speed);
        motors[i]->run(FORWARD);
        total_joint_errors[i] += joint_errors[i];
      }
      else{
        reached_point = reached_point && true;;
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        total_joint_errors[i] = 0.0;
      }
    }
    last_arduino_time = current_arduino_time;
  }

  for(int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i]->setSpeed(0);
      motors[i]->run(RELEASE);
      total_joint_errors[i] = 0.0;
    }
//  Serial.println("Moved t/o New Position");
//  Serial.println('~');/
}

//########################### STOP OR RESET FUNCTIONS ######################################3
void resetJoints(){
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    new_joint_positions[i] = 0.0;
  }
  writeJointPositions();
}

void stop(){
  // Turn off all motors
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i]->run(RELEASE);
  }
}
