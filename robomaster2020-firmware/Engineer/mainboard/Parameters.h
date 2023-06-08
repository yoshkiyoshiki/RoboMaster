// #pragma once
#ifndef __PARAMETERS__
#define __PARAMETERS__

#define MOTOR_NUM 6
#define SUBBORD_MOTOR_NUM 8

double PID_para[MOTOR_NUM][3][3]={
  {           //y_motor
    {1.0, 0.0, 0.0},  //mec(Kp,Ki,Kd)
    {3.0, 0.0, 0.0},  //speed(Kp,Ki,Kd)
    {0.04, 0.0, 0.0}  //position(Kp,Ki,Kd)
  },
  {           //z_right
    {1.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {0.04,0.0,0.0}
  },
  {           //z_left
    {1.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {0.04, 0.0, 0.0}
  },
  {           //arm
    {1.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {0.04, 0.0, 0.0}
  },
  
  {           //eight_joint
    {1.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {0.04, 0.0, 0.0}
  },
  
  {           //left_joint
    {1.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {0.04, 0.0, 0.0}
  }
};

#endif
