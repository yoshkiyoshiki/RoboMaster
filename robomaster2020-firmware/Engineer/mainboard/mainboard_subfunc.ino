void setNeutralAll()
{
  for(int i = 0; i < MOTOR_NUM; i++)
  {
     motor[i].setNeutral(motor[i].getPosition());
  }
}

void moveMotorAll()
{
  for(int i = 0; i < MOTOR_NUM; i++)  
  {
    canmanager->setCAN1C610620Ampere(i+1, ampare[i]);
  }
}

void moveCamera()
{
    canmanager->setCAN2C610620Ampere(7, ampare[camera_motor]);
}

void getM3508Datas(){
 if(canmanager->isNoCan1())  //Chassis
      return;

  uint8_t data[8];
  for(int i=0; i<MOTOR_NUM; i++){
    canmanager->getCan1Data(0x201+i,data);

    motor[i].setPosition( (data[0]<<8) + data[1]);
    motor[i].setRPM( (data[2]<<8) + data[3]);
    motor[i].setCurrent( (data[4]<<8) + data[5]);
    motor[i].setTemperature( data[6]);
  }
}

void getM2006Datas(){
  if(canmanager->isNoCan2())
    return;

  uint8_t data[8];

  canmanager->getCan2Data( 0x208, data);
  CameraMotor.setPosition( (data[0]<<8) + data[1]);
  CameraMotor.setRPM( (data[2]<<8) + data[3]);
  CameraMotor.setCurrent( (data[4]<<8) + data[5]);
  CameraMotor.setTemperature( data[6]);
}

void setPIDParaAll()
{
  for(int i = 0; i < MOTOR_NUM; i++)//各種PID値を設定
  {
    motor[i].setPIDgain_mec(PID_para[i][mec_para][kp], PID_para[i][mec_para][ki], PID_para[i][mec_para][kd]);
    motor[i].setPIDgain_speed(PID_para[i][speed_para][kp], PID_para[i][speed_para][ki], PID_para[i][speed_para][kd]);
    motor[i].setPIDgain_position(PID_para[i][position_para][kp], PID_para[i][position_para][ki], PID_para[i][position_para][kd]);
  }
}

void setPIDForSirahadori()
{
  motor[arm_motor].setPIDgain_mec(2.0, 0.2, 0);
  motor[arm_motor].setPIDgain_speed(8.0, 0, 0);
  motor[arm_motor].setPIDgain_position(0.2, 0, 0);
}
