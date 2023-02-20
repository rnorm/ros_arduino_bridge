#ifdef L298_MOTOR_DRIVER
#define RIGHT_MOTOR_BACKWARD 5
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 9
#define LEFT_MOTOR_FORWARD 10
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_ENABLE 13
#endif

#ifdef YELLOW_JACKET_DRIVER
#define LEFT_FRONT_MOTOR 2
#define RIGHT_FRONT_MOTOR 3
#define LEFT_REAR_MOTOR 4
#define RIGHT_REAR_MOTOR 5
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftFrontSpeed, int rightFrontSpeed, int leftRearSpeed, int rightRearSpeed);
