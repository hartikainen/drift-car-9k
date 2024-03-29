#define SCREEN_LOOP_COUNT 10000
#define STEERING_LOOP_COUNT 100
#define FINISHLINE_LOOP_COUNT 100
#define RPM_LOOP_COUNT 1000
#define BTN_LOOP_COUNT 5000
#define LAPTIME_LOOP_COUNT 781
#define PREV_BP_COUNT 3
#define LEFT_TURN_MASK 0b11100000
#define STRAIGHT_MASK 0b00011000
#define RIGHT_TURN_MASK 0b00000111
#define LEFT_STEERING 0b00000100
#define STRAIGHT_STEERING 0b00000010
#define RIGHT_STEERING 0b00000001

uint8_t* get_track_info(void);
uint8_t get_prediction(int delta);
