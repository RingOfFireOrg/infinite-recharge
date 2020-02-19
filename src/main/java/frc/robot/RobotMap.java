package frc.robot;

public class RobotMap {

	// Ports not on robot
	// Joysticks
	public static final int JOYSTICK_DRIVE_RIGHT = 0;
	public static final int JOYSTICK_DRIVE_LEFT = 1;
	public static final int JOYSTICK_MANIPULATOR = 2;

	//Joystick Buttons

	// Robot Ports
	// Analog Ports

	// CAN Ports
	public static final int NEO_FRONT_LEFT = 1;
	public static final int NEO_FRONT_RIGHT = 2;
	public static final int NEO_BACK_RIGHT = 3;
	public static final int NEO_BACK_LEFT = 4;
	public static final int INTAKE_MOTOR = _;
	public static final int INDEXER_MOTOR = _;
	public static final int SHOOTER_FEEDER_MOTOR = _;
	public static final int SHOOTER_OUTPUT_MOTOR = _;
	public static final int CLIMBER_WINCH = _;
	public static final int CLIMBER_EXTENSION = _;
	public static final int CLIMBER_TRAVERSE = _;

	// Drive Train Encoders
	public static final int DRIVE_TRAIN_LEFT_ENCODER_A = 0;
	public static final int DRIVE_TRAIN_LEFT_ENCODER_B = 1;
	public static final int DRIVE_TRAIN_RIGHT_ENCODER_A = 2;
	public static final int DRIVE_TRAIN_RIGHT_ENCODER_B = 3;

	//design constants:
	public static final double ROBOT_TRACK_WIDTH_IN = 28; //to be defined later
	public static final double DRIVEBASE_GEAR_RATIO = 10.6; //to be defined later
	public static final double DRIVEBOX_KS_CONSTANT = 0.268; //to be defined later
	public static final double DRIVEBOX_KV_CONSTANT = 1.89; //to be defined later
	public static final double DRIVEBOX_KA_CONSTANT = 0.243;//to be defined later
}