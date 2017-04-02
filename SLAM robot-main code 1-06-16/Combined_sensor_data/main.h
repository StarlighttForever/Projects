

enum STATE {
	INITIALISING,
	FIND_WALL,
	TURN_90_CW,
	ALIGN_WALL_ROT,
	ALIGN_WALL_MOV,
	GO_TO_CORNER,
	PRE_MOV_ALIGN,
  OBSTACLE_AVOIDANCE,
	PATHING,
	RUNNING,
	STOPPED
};

class arena {	
public: 
	float Width; //Contains information about the arena
	float Length;
	int index;
	float Size[4]; 
	int loop_count;
	int corner_count;
};

class timer {
public:
	bool started;
	float start_time;
	float elapsed;
};

class control {
public:
	double Setpoint, Input, Output;
	double aggKp, aggKi, aggKd; //aggresive gains
	double consKp, consKi, consKd; //Conservative gains
};

//MOTOR SETUP
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 48;
const byte right_front = 49;
int speed_val = 90;
int speed_change;

#define L_F_Motor 1
#define L_R_Motor 2
#define R_F_Motor 3
#define R_R_Motor 4

Servo left_font_motor;  // create servo object to control a servo
Servo left_rear_motor;  // create servo object to control a servo
Servo right_rear_motor;  // create servo object to control a servo
Servo right_font_motor;  // create servo object to control a servo



//IR ARRAYS SETUP
Average<float> L_M_IR(10);
Average<float> R_M_IR(10);
Average<float> B_L_IR(10);
Average<float> F_L_IR(10);
float L_M_IR_D = 0;
float R_M_IR_D = 0;
float B_L_IR_D = 0;
float F_L_IR_D = 0;
int timer_counter = 0;

//SONAR SETUP
#define MAX_DISTANCE 250 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 30 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
unsigned long pingTimer;
unsigned int cm;         // Where the ping distances are stored.
float Voltage = 0;

//MPU SETUP
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_I2C_ADDRESS        0x68
#define LED_PIN 13

bool blinkState = false;
uint8_t i2cData[14]; // Buffer for I2C data

double MPU_timer;
double GB; // Calculated rotating angle by gyroscope z axis
double gyroZrate;
