package frc.robot;

public class RobotMap {

	// Ports not on robot
	// Joysticks
	public static final int JOYSTICK_DRIVE_LEFT = 0;
	public static final int JOYSTICK_DRIVE_RIGHT = 1;
	public static final int GAMEPAD_MANIPULATOR = 2;

	//Gamepad Buttons
	public static final int MANIPULATOR_LEFT_BUMPER = 5;
	public static final int MANIPULATOR_RIGHT_BUMPER = 6;
	public static final int MANIPULATOR_A_BUTTON = 1;
	public static final int MANIPULATOR_B_BUTTON = 2;
	public static final int MANIPULATOR_X_BUTTON = 3;
	public static final int MANIPULATOR_Y_BUTTON = 4;

	//Gamepad Axes
	public static final int MANIPULATOR_RIGHT_TRIGGER = 3;
	public static final int MANIPULATOR_LEFT_TRIGGER = 2;
	public static final int MANIPULATOR_LEFT_JOYSTICK_Y = 1;
	public static final int MANIPULATOR_RIGHT_JOYSTICK_Y = 5;


	// Robot Ports
	// Analog Ports

	// CAN Ports
	public static final int DT_LEFT_FORWARD = 1; //TODO define this
	public static final int DT_LEFT_BACK = 2;//TODO define this
	public static final int DT_RIGHT_FORWARD = 3;//TODO define this
	public static final int DT_RIGHT_BACK = 4;//TODO define this
	
	public static final int INTAKE_MOTOR = 7;
	public static final int SHOOTER_FEEDER_MOTOR = 5;
	public static final int SHOOTER_OUTPUT_MOTOR = 6;
	public static final int CLIMBER_WINCH = 9;//TODO define this
	public static final int CLIMBER_EXTENSION = 10;//TODO define this
	public static final int CLIMBER_TRAVERSE = 11;//TODO define this
	public static final int CONTROL_PANEL_ACTUATOR = 12; //TODO define this
	public static final int CONTROL_PANEL_SPIN = 13; //TODO define this

	//PWM Ports
	public static final int INDEXER_MOTOR = 0;

	// Drive Train Encoders
	public static final int DRIVE_TRAIN_LEFT_ENCODER_A = 0;
	public static final int DRIVE_TRAIN_LEFT_ENCODER_B = 1;
	public static final int DRIVE_TRAIN_RIGHT_ENCODER_A = 2;
	public static final int DRIVE_TRAIN_RIGHT_ENCODER_B = 3;

	//design constants:
	public static final double ROBOT_TRACK_WIDTH_IN = 22; 
	public static final double DRIVEBASE_GEAR_RATIO = 10.6; 
	public static final double DRIVEBOX_KS_CONSTANT = 0.268; //to be defined later
	public static final double DRIVEBOX_KV_CONSTANT = 1.89; //to be defined later
	public static final double DRIVEBOX_KA_CONSTANT = 0.243;//to be defined later
}