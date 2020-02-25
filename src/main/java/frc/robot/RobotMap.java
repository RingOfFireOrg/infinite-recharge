package frc.robot;

public class RobotMap {

	// Ports not on robot
	// Joysticks
	public static final int JOYSTICK_DRIVE_RIGHT = 0;
	public static final int JOYSTICK_DRIVE_LEFT = 1;
	public static final int GAMEPAD_MANIPULATOR = 2;

	//Joystick Buttons

	// Robot Ports
	// Analog Ports

	// CAN Ports
	public static final int NEO_FRONT_LEFT = 1; //TODO define this
	public static final int NEO_FRONT_RIGHT = 2;//TODO define this
	public static final int NEO_BACK_RIGHT = 3;//TODO define this
	public static final int NEO_BACK_LEFT = 4;//TODO define this
	public static final int INTAKE_MOTOR = 7;
	public static final int INDEXER_MOTOR = 6;//TODO define this
	public static final int SHOOTER_FEEDER_MOTOR = 6;
	public static final int SHOOTER_OUTPUT_MOTOR = 5;
	public static final int CLIMBER_WINCH = 9;//TODO define this
	public static final int CLIMBER_EXTENSION = 10;//TODO define this
	public static final int CLIMBER_TRAVERSE = 11;//TODO define this

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