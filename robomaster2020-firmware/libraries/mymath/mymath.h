#ifndef __mymath_h__
#define __mymath_h__
#include <Arduino.h>

#define RAD 180.0
#define ENCODER_ONE_AROUND 8192
#define M3508_GEAR_RATIO 19.20
#define MATH_E 2.71828182846

#define ABS(num) ((num>=0)? num : -num)
#define sign(num) ((num>0)-(num<0))
#define MIN2(a,b) ((a>b)?b:a)
#define MIN3(a,b,c) MIN2((MIN2(a,b)),c)
#define MAX2(a,b) ((a<b)?b:a)
#define MAX3(a,b,c) MAX2((MAX2(a,b)),c)
#define MEAN(a,b,c) ((a+b+c)-MAX3(a,b,c)-MIN3(a,b,c))
#define MAX4(a,b,c,d) (MAX2(MAX2(a,b),MAX2(c,d)))
#define MIN4(a,b,c,d) (MIN2(MIN2(a,b),MIN2(c,d)))

double dmap(double target, double fromMin, double fromMax, double toMin, double toMax);

long lmap(long target, long fromMin, long fromMax, long toMin, long toMax);

double rad2eular(double rad);
double eular2rad(double eular);

double rad2encoder(double rad);

double encoder2rad(double encoder);

double eular2encoder(double eular);

double encoder2eular(double encoder);

double rpm2radpersec(double _rpm);

double radpersec2rpm(double _radpersec);

class integrate{
private:
    // 双一次変換に必要なデジタルフィルタのパラメータ
    double deltaT;
    double a_1 = 1.0;
    double b_0;
    double b_1;
    // 計算に必要なもの
    double y_1;
    double x_1;
public:
    integrate(double _deltaT);
    integrate(double _a1, double _b0, double _b1);

    double calc_integral(double input);

    ~integrate();
};

class LowPassFilter
{
private:
  double dt;
  double a;

  double prev_input;
  double prev_output;
public:
  LowPassFilter(double deltaT, double riseTime);
  ~LowPassFilter();

  double filter(double input);
};

class PIDController
{
private:
  double pgain, igain, dgain;
  double prev_error;
  double integrate;
  double dt;
  double prev_diff;
public:
  PIDController( double _dt);
  ~PIDController();

  void setGain( double _Pgain, double _Igain, double _Dgain);
  double compute( double error);
};


// プロポのトグル型の定義
typedef enum enm {
  Far = 1,
  Near,
  Center
} RecieverToggle;

typedef union {
  uint8_t bytes[8];
  double value;
} bytes2double;

typedef union {
  uint8_t bytes[4];
  float value;
} bytes2float;

double sigmoid( double gain, double x);
#endif