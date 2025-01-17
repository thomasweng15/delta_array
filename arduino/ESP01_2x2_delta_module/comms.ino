//############################ SERIAL COMM FUNCTIONS #######################################3
void printIPAddr(){
  while (Serial1.available() > 0){
    char ch = Serial1.read();
    Serial.write(ch);
  }
}

void recvWithStartEndMarkers() {
  byte rc;
  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();

    if (recvInProgress == true) {
      if (rc == endMarker) {
//        Serial.println(rc);
        delay(10);
        rc = Serial1.read();
        if (rc == confMarker) {
//          Serial.println(rc);
          delay(10);
          rc = Serial1.read();
//          Serial.println(rc);
          if (rc == confMarker) {
            input_cmd[ndx] = '\0'; // terminate the string
            recvInProgress = false;
            newData = true;
            for (int i=0; i<ndx; i++){
//              Serial.print(input_cmd[i]);Serial.print(" ");
            }
//            Serial.println();
            
          }else{
            input_cmd[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
              ndx = numChars - 1;
            }
          }
        } else{
          input_cmd[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
        }
      }
      else {
        input_cmd[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
    }

    else if (rc == startMarker) {
      delay(10);
      rc = Serial1.read();
      if (rc == confMarker) {
        delay(10);
        rc = Serial1.read();
        if (rc == confMarker) {
          recvInProgress = true;
        }
      }
    }
  }
}

void sendJointPositions(){
  pb_ostream_t ostream = pb_ostream_from_buffer(input_cmd, sizeof(input_cmd));
  message.id = MY_ID;
  for(int i = 0; i < NUM_MOTORS; i++){
    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]); 
    message.joint_pos[i] = motor_val[i] * 0.00006; // 100mm / 1650
//    Serial.print(message.joint_pos[i]);
  }
  if (pb_encode(&ostream, DeltaMessage_fields, &message)){
    Serial1.print("AT+CIPSEND=0,");Serial1.println(ostream.bytes_written +8);
    delay(10);
    printIPAddr();
    delay(10);
    Serial1.write("~~~");
    Serial1.write(input_cmd, ostream.bytes_written);
    Serial1.write("~~~\r\n");
  }
}

bool decodeNanopbData(){  
  pb_istream_t istream = pb_istream_from_buffer(input_cmd, ndx);
  bool ret = pb_decode(&istream, DeltaMessage_fields, &message);
  if (message.id == MY_ID){
    if (message.request_done_state){
        sendJointPositions();
    }
    else if (message.reset){
      for (int i=0; i<NUM_MOTORS; i++){
        new_joint_positions[i] = 0.05;
      }
    }
    else{
      for (int i=0; i<NUM_MOTORS; i++){
        new_joint_positions[i] = message.joint_pos[i];
      }
    }
  }
  else{
    ret = false;
  }
  return ret;
}
