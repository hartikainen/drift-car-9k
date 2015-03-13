#define BUMPER_PORT PORTA
#define BUMPER_DDR DDRA
#define BUMPER_PIN PINA

#define WHEELS_MIN 295
#define WHEELS_MAX 455
#define WHEELS_STEP ((WHEELS_MAX - WHEELS_MIN) / 8)
#define WHEELS_MIDDLE ((WHEELS_MAX + WHEELS_MIN) / 2)

void setup_bumper_ddr(void);
void read_bumper_turn_wheels(void);
void turn_wheels(float direction);
void setup_pwm(int val);
void reset_PID_stuff(void);
