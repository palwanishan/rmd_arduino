
#include <mcp_can.h>
#include <SPI.h>

#define TICK2RAD 2607.4354 
#define CAN0_INT 2      // Set INT to pin 2
MCP_CAN CAN0(10);       // Set CS to pin 

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
long rx[8];

const float amplitute{2};     // change this variable for amplitute in rad !!!
const float frequency{0.5};   // change this variable for freq

// RMD L 7045 = 0.53
// RMD L 5015 = 0.23
const float torque_constant{0.23};    // change this variable depending on your motor !!!
const float gear_ratio{1};
// actuator direction is = 1 or -1
const int actuator_direction{1}; 
const float k_p{2.0};                 // change this variable for tuning gain P
const float k_d{0.1};                 // change this variable for tuning gain D

bool first_time_got_theta{true};
int fb_temp;
float fb_torque;
float fb_speed;
float fb_theta;
float temp_theta_last;
float zero_offset{0};

unsigned long current_time, previous_time{0};


void setup() {
  Serial.begin(115200); delay(4000);
  // Initialize MCP2515 running at 16MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}


void loop() {
  float ref_theta = getTrajectory();
  float tau = getTorqueJointSpacePD(ref_theta, fb_theta, fb_speed);
  setTorque(tau);
  Serial.print("theta: "); Serial.print(fb_theta);
  Serial.print("    ref_theta: "); Serial.print(ref_theta);
//  Serial.print("    fb_speed: "); Serial.print(fb_speed);
//  Serial.print("    tau: "); Serial.print(tau);
  Serial.println();
}


void setTorque(float tau){
  int torque_command_value = tau * 62.5 / torque_constant;
  byte data[8] = {0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  data[4] = torque_command_value & 0xFF;
  data[5] = (torque_command_value >> 8) & 0xFF;
  byte sndStat = CAN0.sendMsgBuf(0x141, 0, 8, data);
  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    for (byte i = 0; i < len; i++) rx[i] = rxBuf[i];
    if(rx[0] == 0xA1){
      fb_temp = rx[1];
      int temp_torque = rx[3] * 256 + rx[2];
      fb_torque = temp_torque * torque_constant / 62;
      int temp_speed = rx[5] * 256 + rx[4];
      fb_speed = temp_speed * 0.01 * actuator_direction / gear_ratio;
      int temp_encoder = (rx[7] * 256 + rx[6]);
      float temp_theta = temp_encoder / TICK2RAD;
//      Serial.print("theta_absolute: "); Serial.println(temp_theta);
      float theta_incremental;
      if(first_time_got_theta){
        fb_theta = zero_offset;
        temp_theta_last = temp_theta;
        theta_incremental = 0;
        first_time_got_theta = false;
      }
      else{
        theta_incremental = temp_theta - temp_theta_last;
        temp_theta_last = temp_theta;
      }
      if(theta_incremental > 4) theta_incremental -= 2*PI;
      else if(theta_incremental < -4) theta_incremental += 2*PI;
      fb_theta += theta_incremental / gear_ratio  * actuator_direction;
//      Serial.print("theta_incremental: "); Serial.println(theta_incremental);
    }
  }
}


void setPosition(){
  
}


float getTrajectory(){
  current_time = millis();
  float current_time_s = current_time * 0.001;
  float ref_theta = amplitute * sin(PI*(frequency * current_time_s));
  return ref_theta;
}


float getTorqueJointSpacePD(float theta_desired, float theta_actual, float theta_dot){
  float tau = k_p * (theta_desired - theta_actual) - k_d * theta_dot;
  return tau;
}
