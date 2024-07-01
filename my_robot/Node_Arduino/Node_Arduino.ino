#include <ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <Wire.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#define QUAYDUONG_R 4
#define QUAYAM_R 5

#define QUAYDUONG_L 6
#define QUAYAM_L 7


#define PI 3.14159265359
#define ADDRESS 0x60 //defines address of compass


int pulse_r = 0;
int pulse_l = 0;
int pulse_l_before = 0;
int pulse_l_after  = 0;
int pulse_r_before = 0;
int pulse_r_after  = 0;

const int R = 0.08;
float x     = 0.0;
float y     = 0.0;
float x_real = 0.0;
float y_real = 0.0;
float heading = 0;
int the_ta_goal = 0;
float dis_tmp      = 0;
int esp          = 0;
float x_goal = 0.0;
float y_goal = 0.0;
float x_update = 0.0;
float y_update = 0.0;
int flag_move = 0;
int flag_turn = 0;
float the_ta_move= 0;
float flag_update = 0;
float flag_heading = 0;
int flag_stop = 0;

//PID steering
unsigned long time;
//const float Kp_the_ta = 1.8333;
//const float Ki_the_ta = 0.23;
//const float Kd_the_ta = 0.08;
const float Kp_the_ta = 1.07;
const float Ki_the_ta = 0.24;
const float Kd_the_ta = 0.08;

int del_ta     = 0;
int the_ta_set = 0;
int PWM_R_set = 60;
int PWM_L_set = 60;
int sum_error_the_ta = 0;
int et_the_ta        = 0;
int es_the_ta        = 0;

unsigned long time_start = 0;

// ROS 
ros::NodeHandle nh;
geometry_msgs::PointStamped pose_head;
geometry_msgs::Point pose_msg;
geometry_msgs::Point flag_msg;

std_msgs::Header h;

ros::Publisher pose_pub("odometry_pose", &pose_msg);
//ros::Publisher flag_pub("flag", &pose_msg);

//ros::Publisher pose_pub2("odometry_pose", &pose_head);
//ros::Subscriber<geometry_msgs::Point> pose_goal_sub("goal_point", &pose_goal_Callback, 1000);
//ros::Subscriber<geometry_msgs::Point> ekf_sub("ekf_pose", &ekf_pose_Callback, 1000);

void count_r()
{
  if (digitalRead(43) == LOW)
  {
     pulse_r ++;
  }
  else
  {
    pulse_r --;
  }
}

void count_l()
{
  if (digitalRead(42) == LOW)
  {    
    pulse_l --;
  }
  else
  {
    pulse_l ++;
  }
}

void run_r(int power_r)
{
  if (power_r >= 0)
  {
    analogWrite(QUAYAM_R, 0);
    analogWrite(QUAYDUONG_R, power_r);
  }
  if (power_r < 0)
  { 
    analogWrite(QUAYDUONG_R, 0);
    analogWrite(QUAYAM_R, power_r);
  }
}

void run_l(int power_l)
{
  if (power_l >= 0)
  {
    analogWrite(QUAYAM_L, 0);
    analogWrite(QUAYDUONG_L, power_l);
  }
  if (power_l < 0)
  { 
    analogWrite(QUAYDUONG_L, 0);
    analogWrite(QUAYAM_L, power_l);
  }
}

// Quay
void Turn_left( int the_ta_goal,int the_ta)
{
  esp = the_ta - the_ta_goal;
  if(abs(esp) > 5)
  {
    run_r(110);
    run_l(-110);
  }
  else{
    flag_turn ++;
    run_r(0);
    run_l(0);
  }
}

void Turn_right( int the_ta_goal,int the_ta)
{
  esp = the_ta -the_ta_goal ;
  if(abs(esp) > 5)
  {
    run_r(-110);
    run_l(110);
  }
  else{
    flag_turn ++;
    run_r(0);
    run_l(0);
  }
}


// Di thang
void Move_Forward(float x_goal, float y_goal, int the_ta)
{
    dis_tmp = sqrt(pow((x-x_goal),2)+pow((y-y_goal),2));
    the_ta_move = (180/PI *atan2((y_goal- y),(x_goal - x))+ 180) ;

    if(the_ta_move > 360)
    {
      the_ta_move = the_ta_move - 360;
     }
    if(the_ta_move < -360)
    {
      the_ta_move = the_ta_move + 360;
    }

    the_ta_move=the_ta_move-180;

 
    if(dis_tmp > 0.03)
    {
      PID_steering(the_ta_move,the_ta);
    }
    else{
      flag_move ++;
      run_r(0);
      run_l(0);
    }
}


void PID_steering(int the_ta_goal,int the_ta)
{
  time = millis();
  the_ta_set = the_ta_goal;
  if(time-time_start >= 100)
  {
    del_ta =the_ta_set- the_ta;
    if(del_ta >= 180)
    {
     del_ta=del_ta-360;
    }
    if(del_ta <= -180)
    {
      del_ta = del_ta + 360;
    }
    sum_error_the_ta += del_ta;

    es_the_ta  = del_ta;
     
    PWM_R_set  = 120 + Kp_the_ta * del_ta + Ki_the_ta * sum_error_the_ta + Kd_the_ta * (es_the_ta - et_the_ta)/0.1;
    PWM_L_set  = 120 - Kp_the_ta * del_ta - Ki_the_ta * sum_error_the_ta - Kd_the_ta * (es_the_ta - et_the_ta)/0.1;

    if(PWM_R_set < 120)
    {
      PWM_R_set = 120;
    }
    if(PWM_R_set > 255)
    {
      PWM_R_set = 255;
    }
    if(PWM_L_set < 120)
    {
      PWM_L_set = 120;
    }
    if(PWM_L_set > 255)
    {
      PWM_L_set = 255;
    }
    time_start = time ;

    et_the_ta = es_the_ta;
  }
      run_r(PWM_R_set);
      run_l(PWM_L_set);
}
void ekf_pose_Callback(const geometry_msgs::Point& ekf_pose_msg)
{
    x_update = ekf_pose_msg.x;
    y_update = ekf_pose_msg.y;
    flag_update = ekf_pose_msg.z;
}

void init_Callback(const geometry_msgs::Point& init_msg)
{
    flag_stop = init_msg.x;
}

ros::Subscriber<geometry_msgs::Point> ekf_sub("ekf_pose", &ekf_pose_Callback);

ros::Subscriber<geometry_msgs::Point> init_sub("decision_node", &init_Callback);


void setup()
{
  nh.initNode();
  nh.advertise(pose_pub);
  nh.subscribe(ekf_sub);
  nh.subscribe(init_sub);

  attachInterrupt(4, count_r, RISING); 
  attachInterrupt(5, count_l, RISING);
  Wire.begin();
  Serial.begin(57600);
}

void loop()
{
  byte highByte;
  byte lowByte;
  
   Wire.beginTransmission(ADDRESS);      //starts communication with cmps03
   Wire.write(2);                         //Sends the register we wish to read
   Wire.endTransmission();

   Wire.requestFrom(ADDRESS, 2);        //requests high byte
   while(Wire.available() < 2);         //while there is a byte to receive
   highByte = Wire.read();           //reads the byte as an integer
   lowByte = Wire.read();
   double yaw = ((highByte<<8)+lowByte)/10; 

   if(flag_heading == 0)
   {
      heading = yaw;
      flag_heading = 1;
   }

   double the_ta = -yaw+heading;

   if(the_ta >=180)
   {
      the_ta -=360;
   }
   if(the_ta <-180)
   {
      the_ta +=360;
   }
   
 //_________________________ODOMETRY_______________________________________
    pulse_l_after = pulse_l;
    pulse_r_after = pulse_r;
    float d_center = (((pulse_l_after - pulse_l_before) + (pulse_r_after - pulse_r_before))* 0.036)/200; // (m)
    x              += d_center * cos(the_ta * 0.0174533); // (m)
    y              += d_center * sin(the_ta * 0.0174533); // (m)

    if(flag_update == 1)
    {  
      x = x_update;
      y = y_update;
    }
  
    

      if(flag_move == 0 && flag_turn == 0)
      {
        Move_Forward(1.2, 0,the_ta);
      }
       else if(flag_move == 1 && flag_turn == 1)
    {
      Move_Forward(4.8, -1.2, the_ta);
    }
    else if(flag_move == 2 && flag_turn == 1)
    {
       Turn_right(-180, the_ta);
    }
    else if(flag_move == 2 && flag_turn ==2)
    {
      Move_Forward(0,-1.1, the_ta);
    }
   else if(flag_move == 3 && flag_turn ==2)
    {
      Turn_left(-90, the_ta);
   }else if(flag_move == 3 && flag_turn ==3)
    {
     Move_Forward(0,-2.4, the_ta);
    }else if(flag_move == 4 && flag_turn ==3)
     {
       Turn_left(0, the_ta);
     }else if(flag_move == 4&& flag_turn == 4)
     {
      Move_Forward(4.8,-2.4, the_ta);
    }else if(flag_move == 5 && flag_turn ==4)
    {
      Turn_left(90,the_ta);
    }else if(flag_move == 5 && flag_turn == 5)
    {
      Move_Forward(4.8,-1.2, the_ta);
    }
    else if(flag_move == 6 && flag_turn == 5 )
    {
      Turn_left(-180, the_ta);
    }
    else if(flag_move == 6 && flag_turn ==6)
    {
      Move_Forward(0,-1.2, the_ta);
    }
    else if(flag_move == 7 && flag_turn == 6)
    {
      Turn_right(90, the_ta);
    }else if(flag_move==7 && flag_turn ==7)
    {
      Move_Forward(0,0,the_ta);
    }else if(flag_move == 8 && flag_turn ==7)
    {
      Turn_right(0, the_ta);
    }
    if(flag_move == 8 && flag_turn == 8)
    {
      Move_Forward(4.8, 0,the_ta);
    }

    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.z = the_ta *0.0174533;

    pose_pub.publish(&pose_msg);
 
    pulse_r_before = pulse_r_after;
    pulse_l_before = pulse_l_after;


    nh.spinOnce();
} 
