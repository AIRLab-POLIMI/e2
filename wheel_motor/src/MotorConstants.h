#ifndef MotorConstants_h
#define MotorConstants_h

//Devices
const char TTYS0[] = "/dev/ttyS0";
const char TTYS1[] = "/dev/ttyS1";
const char TTYUSB0[] = "/dev/ttyUSB0";

const char USB0[] = "/dev/ttyUSB0";
const char USB1[] = "/dev/ttyUSB1";

//AIRBoard constants
const std::string MOTOR_DEV_0 = "m0";
const std::string MOTOR_DEV_1 = "m1";
const std::string KICKER = "k";
const std::string PLUS = "+";
const std::string READ_SPEEDS = "r2s\r";
const std::string READ_ENCODERS = "r2e\r";

//Giano constants
//const std::string TAN_SPEED = "TanSpeed";
//const std::string ROT_SPEED = "RotSpeed";

const std::string TAN_SPEED = "cmd_v";
const std::string ROT_SPEED = "cmd_w";

const std::string ACT_TAN_SPEED = "ActTanSpeed";
const std::string ACT_ROT_SPEED = "ActRotSpeed";

const std::string SPEED_MOTOR_SX = "SpeedMotorSx";
const std::string SPEED_MOTOR_DX = "SpeedMotorDx";
const std::string ENCODER_MOTOR_SX = "EncoderMotorSx";
const std::string ENCODER_MOTOR_DX = "EncoderMotorDx";

//Triskar constants
const std::string TAN_X = "TanX";
const std::string TAN_Y = "TanY";
const std::string SPEED_MODULE = "Module";
const std::string SPEED_ANGLE  = "Angle";
const std::string ROT = "Rot";
const std::string SPEED_MOTOR_0 = "SpeedMotor0";
const std::string SPEED_MOTOR_1 = "SpeedMotor1";
const std::string SPEED_MOTOR_2 = "SpeedMotor2";
const std::string ENCODER_MOTOR_0 = "EncoderMotor0";
const std::string ENCODER_MOTOR_1 = "EncoderMotor1";
const std::string ENCODER_MOTOR_2 = "EncoderMotor2";
const std::string ACT_TAN_X_SPEED = "Act_Tan_X_Speed";
const std::string ACT_TAN_Y_SPEED = "Act_Tan_Y_Speed";
//const std::string ACT_ROT_SPEED = "Act_Rot_Speed";
const std::string ROBOT_X = "robot_x";
const std::string ROBOT_Y = "robot_y";
const std::string ROBOT_PHI = "robot_phi";
const std::string HANDS = "Hands";
const std::string HANDS_1_UP = "u1\r";
const std::string HANDS_2_UP = "u2\r";
const std::string HANDS_1_DOWN = "d1\r";
const std::string HANDS_2_DOWN = "d2\r";
const std::string HANDS_DOWN = "d\r";
const std::string HANDS_UP = "u\r";

//Metamorphic constants
const std::string X_SPEED = "TanX";
const std::string Y_SPEED = "TanY";
const std::string OMEGA_SPEED = "Rot";
const std::string SHAPE = "Shape";
const std::string ACT_X_SPEED = "P108\r";
const std::string ACT_Y_SPEED = "P109\r";
const std::string ACT_OMEGA_SPEED = "P110\r";

const std::string DELTA_X = "delta_x";
const std::string DELTA_Y = "delta_y";
const std::string DELTA_OMEGA = "delta_omega";

const std::string DELTA_X_RESEGHE = "P117\r";
const std::string DELTA_Y_RESEGHE = "P118\r";
const std::string DELTA_OMEGA_RESEGHE = "P119\r";

const std::string RESET_ABACO = "P9=1\r";

const std::string ROBOT_MOVEMENT = "movement";
const std::string ROBOT_DIRECTION = "direction";
const std::string ROBOT_ORIENTATION = "orientation";

const std::string KICK = "Kick";

const std::string STATUS = "Status";
#endif
