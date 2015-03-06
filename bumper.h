#define BUMPER_PORT PORTA
#define BUMPER_DDR DDRA
#define BUMPER_PIN PINA

#define WHEELS_MIDDLE 370

void setup_bumper_ddr(void);
void setup_bumper_timer(void);
void reset_bumper_timer(void);
void read_bumper_turn_wheels(void);
void release_steering(void);
