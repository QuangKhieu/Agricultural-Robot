#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>


// Nhận tọa độ của vật
float x_EE = 265; 
float y_EE = 0;
float z_EE = 108; 

struct JointAngles {
  float q1;
  float q2;
  float q3;
};

JointAngles angles;

void callBackFunction (const geometry_msgs::Quaternion &input) {
  x_EE = input.x;
  y_EE = input.y;
  z_EE = input.z;
}


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Quaternion> sub_cam("goal_pos", &callBackFunction);


// Tọa độ của box
float x_box = 247; 
float y_box = 142;
float z_box = 248; 

// Goc quay hien tai
float q1_c = 0; 
float q2_c = 0;
float q3_c = 0; 


const float pi = 3.14159265359;


// Robot parameters
const float L1 = 90;  // Length of link 1 (mm)
const float L2 = 140;  // Length of link 2 (mm)
const float L3 = 150;  // Length of link 3 (mm)
const float L4 = 90;   // Length of link 4 (mm)

const int switchPin1 = 18;  // Chân đại diện cho công tắc 1
const int switchPin2 = 19; // Chân đại diện cho công tắc 2
const int switchPin3 = 20; // Chân đại diện cho công tắc 3

// Biến trạng thái của từng switch
bool xLimitFlag = false;
bool yLimitFlag = false;
bool zLimitFlag = false;

// Biến trạng thái của từng switch
bool xOriginFlag = false;
bool yOriginFlag = false;
bool zOriginFlag = false;

bool isAtGoal = false;
bool isAtBox = false;

// Định nghĩa các chân kết nối động cơ
const int motor1StepPin = 2;
const int motor1DirPin = 5;
const int motor2StepPin = 3;
const int motor2DirPin = 6;
const int motor3StepPin = 4;
const int motor3DirPin = 7; 

// Khởi tạo đối tượng AccelStepper cho các động cơ
AccelStepper stepper1(AccelStepper::DRIVER, motor1StepPin, motor1DirPin);
AccelStepper stepper2(AccelStepper::DRIVER, motor2StepPin, motor2DirPin);
AccelStepper stepper3(AccelStepper::DRIVER, motor3StepPin, motor3DirPin);

// Khai báo chân kết nối servo
const int servoPin = 31;

// Khởi tạo đối tượng servo
Servo gripperServo;

// Hằng số góc mở và góc đóng của gripper
const int gripOpenAngle = 60;     // Góc mở của gripper
const int gripCloseAngle = 30;   // Góc đóng của gripper

const unsigned long stepInterval = 2000;  // Khoảng thời gian giữa các bước (milliseconds)
unsigned long previousStepTime = 0;

// Biến trạng thái
enum State {
  CHECK_LIMITS,
  MOVE_TO_LIMITS,
  MOVE_TO_ORIGIN,
  MOVE_TO_GOAL,
  CLOSE_GRIP,
  MOVE_TO_BOX,
  OPEN_GRIP
};

State currentState = CHECK_LIMITS; // Trạng thái ban đầu

void setup() {
  Serial.begin(57600); // Khởi tạo kết nối với Serial Monitor

  pinMode(switchPin1, INPUT);
  pinMode(switchPin2, INPUT);
  pinMode(switchPin3, INPUT);

  // Cài đặt tốc độ và gia tốc cho các động cơ
  stepper1.setMaxSpeed(400);
  stepper1.setAcceleration(200);
  stepper2.setMaxSpeed(400);
  stepper2.setAcceleration(200);
  stepper3.setMaxSpeed(400);
  stepper3.setAcceleration(200);

  attachInterrupt(5, reachXLimit, HIGH); // Check limit X
  attachInterrupt(3, reachZLimit, HIGH); // Check limit Z
  attachInterrupt(4, reachYLimit, HIGH); // Check limit Y

  // Cấu hình chân servo
  gripperServo.attach(servoPin);

  // Thiết lập trạng thái mở ban đầu của gripper
  gripperServo.write(gripCloseAngle);

  nh.initNode();
  nh.subscribe(sub_cam);
}

void loop() {

  switch (currentState) {
    case CHECK_LIMITS:
      checkSwitchs();

      // Di chuyển robot đến vị trí giới hạn
      if(!xLimitFlag) {
        moveToXLimits();
      }
      
      if(!zLimitFlag) {
        moveToZLimits();
      }

      if(!yLimitFlag && zLimitFlag) { // 
        moveToYLimits();
      }
      
      // Kiểm tra nếu robot đã đến vị trí giới hạn
      if (xLimitFlag && yLimitFlag && zLimitFlag) {
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
//         q1_c = -90;
//         q2_c = 140;
//         q3_c = -180;
         currentState = MOVE_TO_ORIGIN; // Chuyển sang bước di chuyển đến vị trí gốc
        Serial.println("done limit");
      }
      break;

    case MOVE_TO_ORIGIN:
      // Di chuyển robot từ vị trí giới hạn đến vị trí gốc
      if (!xOriginFlag) { 
        delay(1000);
        move(motor1DirPin, motor1StepPin, LOW, 190, 2000); // (dirPin, stepPin, dir, step, speed)
        xOriginFlag = true;
        q1_c = 0;
      }
      if (!yOriginFlag) {
        move(motor2DirPin, motor2StepPin, HIGH, 116, 3000); // 116: 90 dg
        yOriginFlag = true;
        q2_c = 90;
      }
      if (!zOriginFlag) {
        move(motor3DirPin, motor3StepPin, LOW, 92, 2500);
        zOriginFlag = true;
        q3_c = -66;
      }
      
      // Kiểm tra nếu robot đã đến vị trí gốc
      if (xOriginFlag && yOriginFlag && zOriginFlag) {
        isAtGoal = false;
        currentState = MOVE_TO_GOAL; // Chuyển sang bước di chuyển đến vị trí giới hạn
        Serial.println("done origin");
        
      }
//      delay(1000);/
      break;
    case MOVE_TO_GOAL:
      angles = inverseKinematics(x_EE, y_EE, z_EE);
      Serial.println(angles.q1);
      Serial.println(angles.q2);
      Serial.println(angles.q3);
      if (!isAtGoal) {
        moveToGoal(angles);
      } else {
        q1_c = angles.q1;
        q2_c = angles.q2;
        q3_c = angles.q3;
        Serial.println("done goal");
        currentState = CLOSE_GRIP;
      }
        
      break;
    case CLOSE_GRIP:
      Serial.println("start close grip");
      gripperServo.write(gripOpenAngle); // Mở gripper
      delay(1000); 
      
      gripperServo.write(gripCloseAngle); // Đóng gripper
      
      currentState = MOVE_TO_BOX; 
      break;
    case MOVE_TO_BOX:
      angles = inverseKinematics(x_box, y_box, z_box);
      if (!isAtBox) {
        moveToBox(angles);
      } else {
        q1_c = angles.q1;
        q2_c = angles.q2;
        q3_c = angles.q3;
        Serial.println("done box");
        currentState = OPEN_GRIP;
      }
        
      break;
    case OPEN_GRIP:
      gripperServo.write(gripOpenAngle); // Mở gripper
      delay(1000); 
      
      gripperServo.write(gripCloseAngle); // Đóng gripper
      currentState = MOVE_TO_LIMITS; 
      break;
    
    case MOVE_TO_LIMITS:
      isAtBox = false;
      checkSwitchs();

      // Di chuyển robot đến vị trí giới hạn
      if(!xLimitFlag) {
        moveToXLimits();
      }
      
      if(!zLimitFlag) {
        moveToZLimits();
      }

      if(!yLimitFlag && zLimitFlag) { // 
        moveToYLimits();
      }
      
      // Kiểm tra nếu robot đã đến vị trí giới hạn
      if (xLimitFlag && yLimitFlag && zLimitFlag) {
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        break;
      }
      break;

  }
  nh.spinOnce();
}


void moveToGoal(JointAngles angles) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousStepTime >= stepInterval) {
    // Lưu lại thời gian hiện tại để tính toán khoảng thời gian tiếp theo
    previousStepTime = currentMillis;
    
    float xGoalPos = stepper1.currentPosition() + ((angles.q1 - q1_c) / 1.8 * 5);
    float yGoalPos = stepper2.currentPosition() + (-(angles.q2 - q2_c) / 1.8 * 3);
    float zGoalPos = stepper3.currentPosition() + ((-angles.q3 + q3_c - q2_c + angles.q2) / 1.8 * 3);

    stepper1.moveTo(xGoalPos);
    stepper2.moveTo(yGoalPos);
    stepper3.moveTo(zGoalPos);

  }
  
  Serial.println(stepper1.distanceToGo());
  // Kiểm tra nếu đạt được vị trí mục tiêu
  if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
    // Thiết lập cờ hiệu đến nơi
    isAtGoal = true;

    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
  } else {
    isAtGoal = false;
  }
  stepper1.run();
  stepper2.run();
  stepper3.run();
}

void moveToBox(JointAngles angles) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousStepTime >= stepInterval) {
    // Lưu lại thời gian hiện tại để tính toán khoảng thời gian tiếp theo
    previousStepTime = currentMillis;
    
    float xGoalPos = stepper1.currentPosition() + ((angles.q1 - q1_c) / 1.8 * 5);
    float yGoalPos = stepper2.currentPosition() + (-(angles.q2 - q2_c) / 1.8 * 3);
    float zGoalPos = stepper3.currentPosition() + ((-angles.q3 + q3_c - q2_c + angles.q2) / 1.8 * 3);

    stepper1.moveTo(xGoalPos);
    stepper2.moveTo(yGoalPos);
    stepper3.moveTo(zGoalPos);
  }
  
  // Kiểm tra nếu đạt được vị trí mục tiêu
  if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
    // Thiết lập cờ hiệu đến nơi
    isAtBox = true;

    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
  } else {
    isAtBox = false;
  }
  stepper1.run();
  stepper2.run();
  stepper3.run();
}


void moveToXLimits() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousStepTime >= stepInterval) {
    // Lưu lại thời gian hiện tại để tính toán khoảng thời gian tiếp theo
    previousStepTime = currentMillis;
    
    float xLimitPosition = stepper1.currentPosition() + (180 / 1.8 * 5);
    stepper1.moveTo(xLimitPosition);
  }
  stepper1.run(); 
}

void reachXLimit() {
//  Serial.println("reach X");/
  xLimitFlag = true;
  stepper1.stop();
}

void moveToYLimits() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousStepTime >= stepInterval) {
    // Lưu lại thời gian hiện tại để tính toán khoảng thời gian tiếp theo
    previousStepTime = currentMillis;
    
    float yLimitPosition = stepper2.currentPosition() + (-180 / 1.8 * 3);
    stepper2.moveTo(yLimitPosition);
  }
  stepper2.run();

}

void reachYLimit() {
//  Serial.println("reach Y");
  yLimitFlag = true;
  stepper2.stop();
}

void moveToZLimits() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousStepTime >= stepInterval) {
    // Lưu lại thời gian hiện tại để tính toán khoảng thời gian tiếp theo
    previousStepTime = currentMillis;
    
    float zLimitPosition = stepper3.currentPosition() + (180 / 1.8 * 3);
    stepper3.moveTo(zLimitPosition);
  }
  stepper3.run();
}

void reachZLimit() {
//  Serial.println("reach Z");
  zLimitFlag = true;
  stepper3.stop();
}

void move(int dirPin, int stepPin, bool dir, int step, int speed) {
  digitalWrite(dirPin, dir);
  for(int i = 0; i < step; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed);
  }
}


// Inverse kinematics function
JointAngles inverseKinematics(float x_EE, float y_EE, float z_EE) {
  JointAngles angles;

  // Calculate q1
  angles.q1 = atan2(y_EE, x_EE);

  // Calculate position of joint #4
  float x_4 = x_EE - (L4 * cos(angles.q1));
  float y_4 = y_EE - (L4 * sin(angles.q1));
  float z_4 = z_EE;

  // Calculate length of the virtual side
  float z_1 = L1;
  float z_1_4 = z_4 - z_1;
  float xy_4 = sqrt((x_4 * x_4) + (y_4 * y_4));
  float v_side = sqrt((z_1_4 * z_1_4) + (xy_4 * xy_4));

  // Calculate q2
  float q2_a = atan2(z_1_4, xy_4);
  float q2_b = acos((v_side * v_side + L2 * L2 - L3 * L3) / (2 * v_side * L2));
  angles.q2 = q2_a + q2_b;

  if (angles.q2 < 60)  {
    // Calculate q3
    angles.q3 = - acos((L2 * L2 + L3 * L3 - v_side * v_side) / (2 * L2 * L3));  
  } else {
    // Calculate q3
    angles.q3 = - (pi - acos((L2 * L2 + L3 * L3 - v_side * v_side) / (2 * L2 * L3)));
  }

  // Convert angles to degrees
  angles.q1 = (angles.q1 * 180/pi);
  angles.q2 = (angles.q2 * 180/pi);
  angles.q3 = (angles.q3 * 180/pi);

  return angles;
}


void checkSwitchs() {
  bool currentState1 = digitalRead(switchPin1);
  bool currentState2 = digitalRead(switchPin2);
  bool currentState3 = digitalRead(switchPin3);

  if(currentState1 == HIGH) {
    xLimitFlag = true;
  } else {
    xLimitFlag = false;
  }

  if(currentState2 == HIGH) {
    yLimitFlag = true;
  } else {
    yLimitFlag = false;
  }

  if(currentState3 == HIGH) {
    zLimitFlag = true;
  } else {
    zLimitFlag = false;
  }
}
