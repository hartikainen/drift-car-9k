#define BUMPER_PORT PORTA
#define BUMPER_DDR DDRA
#define BUMPER_PIN PINA

/* Values for the front wheel servo */
#define WHEELS_MIN 295
#define WHEELS_MAX 455
#define WHEELS_STEP ((WHEELS_MAX - WHEELS_MIN) / 8)
#define WHEELS_MIDDLE ((WHEELS_MAX + WHEELS_MIN) / 2)

/* Values for PID steering, s for steering */
#define sKp 0.3
#define sKi 0.0
#define sKd 0.0

#define LAP_THRESHOLD 2
#define SAFETY_COUNTER 400 // Counter for detecting out of track

void setup_bumper_ddr(void);
void read_bumper_turn_wheels(uint8_t);
void turn_wheels(int direction);
void setup_pwm(int val);
void reset_PID_stuff(void);
int target_from_bumper_led(uint8_t);
int get_bumper_int(void);
int get_current_lap(void);
int get_laptime_secs(void);
int get_laptime_partial(void);
int get_lap_record_secs(void);
int get_lap_record_partial(void);
int get_lap_record_lap(void);
int get_hamming_weight(uint8_t);
void update_laptime(void);
void reset_laptime(void);
int get_hamming(void);
void check_lap_record(void);
void check_finish_line(void);
