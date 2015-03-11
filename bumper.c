#include <avr/io.h>
#include "bumper.h"

volatile int steering_locked = 0;
int bumper_int;
uint8_t bumper;

void setup_bumper_ddr(void)
{
  BUMPER_DDR = 0;
}

#define Kp = 1;
#define Ki = 0;
#define Kd = 0;
#define TARGET = 0;

int round_int(float r)
{
  // http://stackoverflow.com/a/4660122
  return (r > 0.0) ? (r + 0.5) : (r - 0.5);
}

int log2(int x)
{
  // http://stackoverflow.com/a/9612244
  int ans = 0 ;
  while(x>>=1)
  {
    ans++;
  }
  return ans;
}

int ipow(int exp)
{
  // http://stackoverflow.com/a/101613
  int base = 2;
  int result = 1;
  while (exp)
  {
    if (exp & 1)
    {
      result *= base;
    }
    exp >>= 1;
    base *= base;
  }
  return result;
}

int moving_avg( int x )
{
  // doesn't handle bits like 0b001100000 correctly
  static float old[] = {0.0,0.0,0.0};
  float y;
  if (x == 0)
  {
    // We sampled 0b00000000 but the last sample was not 0b10000000, 0b00000001 or 0b00000000 pad with last sample.
    if (old[2] > 1.0 && old[2] < 7.0)
    {
      y = old[2];
    }
    else
    {
      y = 0.0;
    }
  }
  else
  {
    // calculate log2 of sample because averaging won't work well over values like 128, 64, 32...
    if (x > 1)
    {
      y = log2(x);
    }
    else
    {
      y = x;
    }
  }

  // calculate moving average over 4 measurements
  float new_value = (old[0]+old[1]+old[2]+(float)y)/4;
  old[0] = old[1];
  old[1] = old[2];
  old[2] = y;

  // calculate the power of 2 to return value in the correct form again
  if (new_value != 0.0)
  {
    return ipow(round_int(new_value));
  }
  return 0;
}

void read_bumper_turn_wheels(void)
{
  bumper = ~BUMPER_PIN;
  switch (bumper) {
  case 0b00000001:
    //setup_pwm(WHEELS_MIDDLE - 40);
    bumper_int = -4;
    break;
  case 0b00000010:
    //setup_pwm(WHEELS_MIDDLE - 30);
    bumper_int = -3;
    break;
  case 0b00000100:
    //setup_pwm(WHEELS_MIDDLE - 20);
    bumper_int = -2;
    break;
  case 0b00001000:
    //setup_pwm(WHEELS_MIDDLE - 10);
    bumper_int = -1;
    break;
  case 0b00010000:
    //setup_pwm(WHEELS_MIDDLE + 10);
    bumper_int = 1;
    break;
  case 0b00100000:
    //setup_pwm(WHEELS_MIDDLE + 20);
    bumper_int = 2;
    break;
  case 0b01000000:
    //setup_pwm(WHEELS_MIDDLE + 30);
    bumper_int = 3;
    break;
  case 0b10000000:
    //setup_pwm(WHEELS_MIDDLE + 40);
    bumper_int = 4;
    break;
  default:
    bumper_int = 0;
    //setup_pwm(WHEELS_MIDDLE);
  }
  int servo_value = WHEELS_MIDDLE;

  y = Kp * 
}

