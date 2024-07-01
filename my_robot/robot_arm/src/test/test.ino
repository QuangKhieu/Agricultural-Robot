#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

// Nhận tọa độ của vật 

float x_EE; 
float y_EE;
float z_EE; 

bool isRecv = false;

struct JointAngles {
  float q1;
  float q2;
  float q3;
};

JointAngles angles;

ros::NodeHandle nh;

void callBackFunction (const geometry_msgs::Quaternion &input) {
  isRecv = true;
  x_EE = input.x;
  y_EE = input.y;
  z_EE = input.z;
}

//ros::Subscriber<geometry_msgs::Quaternion> sub_cam("goal_pos", callBackFunction);
geometry_msgs::PointStamped init_flag_msg_header;
geometry_msgs::Point init_flag_msg;
ros::Publisher init_flag_pub("init", &init_flag_msg_header);
std_msgs::Header h;


// Tọa độ của box
float x_box = 190; 
float y_box = -225;
float z_box = 118; 

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
const int servoPin = 21;

// Khởi tạo đối tượng servo
Servo gripperServo;

// Hằng số góc mở và góc đóng của gripper
const int gripOpenAngle = 90;     // Góc mở của gripper
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
  stepper1.setMaxSpeed(200);
  stepper1.setAcceleration(200);
  stepper2.setMaxSpeed(200);
  stepper2.setAcceleration(200);
  stepper3.setMaxSpeed(200);
  stepper3.setAcceleration(200);

  attachInterrupt(5, reachXLimit, HIGH); // Check limit X
  attachInterrupt(3, reachZLimit, HIGH); // Check limit Z
  attachInterrupt(4, reachYLimit, HIGH); // Check limit Y

  // Cấu hình chân servo
  gripperServo.attach(servoPin);

  // Thiết lập trạng thái mở ban đầu của gripper
  gripperServo.write(gripOpenAngle);

  nh.initNode();
  nh.advertise(init_flag_pub);
//  nh.subscribe(sub_cam);
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
        
         currentState = MOVE_TO_ORIGIN; // Chuyển sang bước di chuyển đến vị trí gốc
        Serial.println("done limit");
        delay(2000);
      }
      break;

    case MOVE_TO_ORIGIN:
      // Di chuyển robot từ vị trí giới hạn đến vị trí gốc
      if (!xOriginFlag) { 
        delay(1000);
        move(motor1DirPin, motor1StepPin, LOW, 200, 2000); // (dirPin, stepPin, dir, step, speed)
        xOriginFlag = true;
        q1_c = 0;
      }
      if (!yOriginFlag) {
        move(motor2DirPin, motor2StepPin, HIGH, 150, 3000); // 116: 90 dg
        yOriginFlag = true;
        q2_c = 60;
      }
      if (!zOriginFlag) {
        move(motor3DirPin, motor3StepPin, LOW, 120, 2500);
        zOriginFlag = true;
        q3_c = -66;
      }
      
      // Kiểm tra nếu robot đã đến vị trí gốc
      if (xOriginFlag && yOriginFlag && zOriginFlag) {
        isAtGoal = false;
        currentState = MOVE_TO_GOAL; // Chuyển sang bước di chuyển đến vị trí giới hạn
        Serial.println("done origin");
        delay(100);
      }
      break;
    case MOVE_TO_GOAL:
//      angles = inverseKinematics(x_EE, y_EE, z_EE);
      JointAngles ang1;
      ang1.q1 = -30;
      ang1.q2 = 60;
      ang1.q3 = -107;
      if (!isAtGoal) {
        moveToGoal(ang1);
      } else {
        angles = inverseKinematics(x_EE, y_EE, z_EE);
        q1_c = angles.q1;
        q2_c = angles.q2;
        q3_c = angles.q3;
        Serial.println("done goal");
        isAtGoal = false;
        currentState = CLOSE_GRIP;
      }
        
      break;
    case CLOSE_GRIP:
      Serial.println("start close grip");
//      gripperServo.write(gripOpenAngle); // Mở gripper
      delay(1500); 
      
      gripperServo.write(gripCloseAngle); // Đóng gripper
      
      currentState = MOVE_TO_BOX; 
      break;
    case MOVE_TO_BOX:
//      angles = inverseKinematics(x_box, y_box, z_box);
      JointAngles ang2;
      ang2.q1 = 80;
      ang2.q2 = 30;
      ang2.q3 = -58;
      if (!isAtGoal) {
        moveToGoal(ang2);
      } else {
        q1_c = angles.q1;
        q2_c = angles.q2;
        q3_c = angles.q3;
        isAtGoal = false;
        Serial.println("done box");
        currentState = OPEN_GRIP;
      }
        
      break;
    case OPEN_GRIP:
      gripperServo.write(gripOpenAngle); // Mở gripper
      delay(1500); 
      
//      gripperServo.write(gripCloseAngle); // Đóng gripper//
      currentState = MOVE_TO_LIMITS; 
      break;
    
    case MOVE_TO_LIMITS:
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
         isRecv = false;
        break;
      }
      break;

  }
//  if (xLimitFlag && yLimitFlag && zLimitFlag)
//   
//     {
//      init_flag_msg.x = 1;
//     }
//  else
//  {
//    init_flag_msg.x = 0;
//  }
//  
//   init_flag_msg_header.header = h;
//  init_flag_msg_header.point = init_flag_msg;
////  Serial.println(init_flag_msg.x);
//
//
//  init_flag_pub.publish(&init_flag_msg_header);
  nh.spinOnce();
}


void moveToGoal(JointAngles angles) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousStepTime >= stepInterval) {
    // Lưu lại thời gian hiện tại để tính toán khoảng thời gian tiếp theo
    previousStepTime = currentMillis;
    
    float xGoalPos = stepper1.currentPosition() + ((angles.q1 - q1_c) / 1.8 * 5);
    float yGoalPos = stepper2.currentPosition() + (-(angles.q