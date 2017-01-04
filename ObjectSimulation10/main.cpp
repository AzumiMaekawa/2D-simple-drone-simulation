//
//  main.cpp
//  ObjectSimulation10
//
//  Created by AzumiMaekawa on 2017/01/05.
//  Copyright © 2017年 AzumiMaekawa. All rights reserved.
//




#include <iostream>
#include <math.h>
#include <stdlib.h>
#define MaxPower 0.1


double cnt = 0;
double dt = 0.01;
const int delay_time = 50;
const int StepNum = 100;
double StepSize = MaxPower/StepNum;

double delay[delay_time];

double MotorPowerStep[StepNum];

const int T = 300;




class Object{
public:
    
    double MotorControlTime;
    
    
    //  double x, y;
    
    double l;
    double I;
    
    //    double RightSide_y, LeftSide_y;
    //   double x_accel, y_accel;
    double degZ;
    double degZ_vel;
    double degZ_accel;
    //   double degZ_accel_delay;
    
    double old_degZ;
    
    double MotorPower;
    double MotorPower_delay;
    double MotorInput;
    
    double ImpulseForce;
    double StepForce;
    
    double Kp, Ki, Kd;
    
    double f_right;
    double f_left;
    
    double f_p, f_i, f_d;
    
    double Kp_p, Kp_i, Kp_d;
    double Ki_p, Ki_i, Ki_d;
    double Kd_p, Kd_i, Kd_d;
    
    
    
    
    
    Object(double initial_degZ_vel, double initial_degZ);
    
    
    /*
     
     void Object(double initial_degZ_vel, double initial_degZ){
     x = 0; y = 0;
     l = 1;
     I = 1;
     
     
     degZ = initial_degZ;
     degZ_vel = initial_degZ_vel;
     
     
     RightSide_y = l * sin(degZ);
     LeftSide_y = l * sin(degZ);
     
     
     }*/
    
    void update();
    void pid_update();
    void MotorControl();
    void MotorDelay();
    void printDeg(double time);
    
    void impulse(double torque, double start_time, double time);
    void step(double torque, double start_time, double time);
    void hysteresis();
    
    
    
};

Object::Object(double initial_degZ_vel, double initial_degZ){
    //   x = 0; y = 0;
    l = 1;
    I = 1;
    
    
    degZ = initial_degZ;
    degZ_vel = initial_degZ_vel;
    
    for(int i = 0;i < StepNum;i++){
        MotorPowerStep[i] = StepSize * (i+1);
    }
    
    
    
    //   RightSide_y = l * sin(degZ);
    //   LeftSide_y = l * sin(degZ);
    
    
}


void Object::update(){
    old_degZ = degZ;
    
    degZ_accel = (l/I) * MotorPower + (1/I) * (ImpulseForce + StepForce);
    
    degZ_vel = degZ_vel + degZ_accel * dt;
    degZ = degZ + degZ_vel * dt;
    //    RightSide_y = l * sin(degZ);
    //    LeftSide_y = l * sin(degZ);
}

void Object::pid_update(){
    
    //pidパラメータにpid制御
    
    Kp_p = 0.02*degZ;
    Kp_i = Kp_i + 0.0000000000003*degZ;
    Kp_d = 20*(degZ - old_degZ);
    
    Ki_p = 0.0000000018*degZ;
    Ki_i = Ki_i + 0.000000000002*degZ;
    Ki_d = 0.000003*(degZ - old_degZ);
    
    Kd_p = 9*degZ;
    Kd_i = Kd_i + 0.0000002*degZ;
    Kd_d = 0.004*(degZ - old_degZ);
    
    //絶対値をとる
    Kp = fabs(Kp_p + Kp_i + Kp_d);
    Ki = fabs(Ki_p + Ki_i + Ki_d);
    Kd = fabs(Kd_p + Kd_i + Kd_d);
}

void Object::MotorControl(){
    
    //pid制御
    f_p = Kp * degZ;
    f_i = f_i + Ki * degZ;
    f_d = Kd * (degZ - old_degZ);
    
    
    f_right = -f_p - f_i - f_d;
    
    
    //モーター入力をステップ状にする
    if((-MotorPowerStep[0] < f_right)&&(f_right < MotorPowerStep[0]))
        f_right = 0;
    for(int j = 1; j < StepNum; j++){
        if((MotorPowerStep[j-1] <= f_right)&&(f_right < MotorPowerStep[j])){
            f_right = MotorPowerStep[j-1];
        }
        else if((-MotorPowerStep[j] <= f_right)&&(f_right < -MotorPowerStep[j-1])){
            f_right = -MotorPowerStep[j-1];
        }
    }
    
    
    
    f_left = -f_right;
    
    //最大出力制限
    if(f_right > MaxPower) f_right = MaxPower;
    if(f_right < -MaxPower) f_right = -MaxPower;
    if(f_left > MaxPower) f_left = MaxPower;
    if(f_left < -MaxPower) f_left = -MaxPower;
    
    
    
    MotorPower_delay = f_right - f_left;
    
    
    
    
}

void Object::MotorDelay(){
    
    delay[0] = MotorPower_delay;
    
    for(int i = delay_time - 1; i > 0; i--){
        delay[i] = delay[i-1];
        
    }
    
    MotorInput = delay[delay_time - 1];
    
    
}

void Object::printDeg(double time){
    printf("%f,%f,%f\n", time, degZ, MotorPower_delay);
}

void Object::impulse(double torque, double start_time, double time){
    if((time > start_time - dt)&&(time < start_time + dt))
        ImpulseForce = torque;
    else
        ImpulseForce = 0;
    
}

void Object::step(double torque, double start_time, double time){
    if(time > start_time - dt)
        StepForce = torque;
}

void Object::hysteresis(){
    
    if((MotorInput < 2*MaxPower*0.05)&&(MotorInput > -2*MaxPower*0.05)){
        if((MotorPower < 2*MaxPower*0.001)&&(MotorPower > -2*MaxPower*0.001))
            MotorPower = 0;
        else
            MotorPower = MotorInput;
    }else
        MotorPower = MotorInput;
    
    
}






int main(int argc, const char * argv[]) {
    // insert code here...
    Object  object(0,0);
    
    
    
    for(double t = 0; t < T; t+=dt){
        
        
        object.update();
        object.pid_update();
        object.MotorControl();
        object.MotorDelay();
        object.hysteresis();
        object.impulse(300, 0, t);
        object.step(0.1, 0, t);
        object.printDeg(t);
        
        
    }
    return 0;
}