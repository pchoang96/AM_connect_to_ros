#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define clkw        0
#define c_clkw      1
#define M1_p        6
#define M1_l        7
#define M2_p        5
#define M2_l        4

int M1=0,M2=0;

ros::NodeHandle  nh;

void messageCb(const geometry_msgs::Twist& vel)
{
    
    M1=vel.linear.x;
    M2=vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  pinMode(M1_l,OUTPUT);
  pinMode(M2_l,OUTPUT);
}

void loop()
{
  pwmOut(M2,M1,0,0);
  nh.spinOnce();
  delay(1);
}

void pwmOut(int Lpwm, int Rpwm, bool Ldir, bool Rdir)
{
  if(Lpwm==0 && Rpwm==0)
  { 
    analogWrite(M1_p,0); digitalWrite(M1_l,0);
    analogWrite(M2_p,0); digitalWrite(M2_l,0);
  }
  else if(Ldir==clkw && Rdir==clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }

  else if(Ldir==c_clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0) ;  
  }

  else if(Ldir==c_clkw && Rdir==clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0);  
  }

  else if(Ldir==clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }  
}
