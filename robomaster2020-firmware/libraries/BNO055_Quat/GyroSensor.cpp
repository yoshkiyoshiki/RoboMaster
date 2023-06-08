#include<GyroSensor.h>
#include <stdio.h>

GyroSensor::GyroSensor(Adafruit_BNO055 *bno)
{
  this->bno = bno;
}

float GyroSensor::angleVelo()
{
  return gyro_data;
}

float GyroSensor::roll()
{
  return roll_data;
}

float GyroSensor::pitch()
{
  return pitch_data;
}

float GyroSensor::yaw()
{
  return yaw_data;
}

void GyroSensor::getAxisData(sensors_event_t* event)
{ 
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  {
    //accel
    // Serial.println("");
    //Serial.print("Accl:");
    accel_data[0] = event->acceleration.x;
    accel_data[1] = event->acceleration.y;
    accel_data[2] = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
  {
    //mag
    // Serial.print("Mag:");
    mag_data[0] = event->magnetic.x;
    mag_data[1] = event->magnetic.y;
    mag_data[2] = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
  {
    //gyro
    //Serial.print("Rot:");
    //gyro_data[0] = event->gyro.x;//roll
    gyro_data = event->gyro.y;//pitch
    //gyro_data[2] = event->gyro.z;//yaw
  }
}

void GyroSensor::quaternionToEuler()
{
  bno->getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno->getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno->getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno->getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno->getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  getAxisData(&angVelocityData);
  getAxisData(&magnetometerData);
  getAxisData(&accelerometerData);

  quat = bno->getQuat();

  w_data = quat.w();
  x_data = quat.x();
  y_data = quat.y();
  z_data = quat.z();

  xsqr = x_data * x_data;
  ysqr = y_data * y_data;
  zsqr = z_data * z_data;

  //roll
  t0 = +2.0 * (w_data * x_data + y_data * z_data);
  t1 = w_data * w_data - xsqr - ysqr + zsqr/*+1.0 - 2.0 * (xsqr + ysqr)*/;
  roll_data = atan2(t0, t1);

  //pitch
  t2 = +2.0 * (w_data * y_data - z_data * x_data);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch_data = asin(t2);

  //yaw
  t3 = +2.0 * (w_data * z_data + x_data * y_data);
  t4 = w_data * w_data + xsqr - ysqr - zsqr /*+1,0 - 2.0 - (ysqr + zsqr)*/;
  yaw_data = atan2(t3, t4);

  roll_data *= 57.2957795131;
  pitch_data *= 57.2957795131;
  yaw_data *= 57.2957795131;

  system = 0;
  gyro = 0;
  accel = 0;
  mag = 0;

  bno->getCalibration(&system, &gyro, &accel, &mag);
}

void GyroSensor::start()
{
  if(!bno->begin())
  {
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}
