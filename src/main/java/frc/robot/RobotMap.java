package frc.robot;

public class RobotMap {

	// Ports not on robot
	// Joysticks
	public static final int JOYSTICK_DRIVE_RIGHT = 0;
	public static final int JOYSTICK_DRIVE_LEFT = 1;
	public static final int JOYSTICK_MANIPULATOR = 2;

	//Joystick Buttons

	public static final int TRANSFER_FORWARD_BTN = 5;
	public static final int TRANSFER_BACKWARD_BTN = 4;
	public static final int INTAKE_INPUT_BTN = 3;
	public static final int INTAKE_OUTPUT_BTN = 2;

	// Robot Ports
	// Analog Ports

	// Drive Train Motors
	//
	public static final int NEO_FRONT_LEFT = 1;
	public static final int NEO_FRONT_RIGHT = 3;
	public static final int NEO_BACK_RIGHT = 4;
	public static final int NEO_BACK_LEFT = 2;

	
	//Bottom Shooter Wheel

	public static final int MOTOR_SHOOTER = 5;

	// Top Shooter Wheel

	public static final int MOTOR_SHOOTER2 = 6;
	
	// Intake

	public static final int CAN_INTAKE = 7;

	// Whole Wheat Transfer

	public static final int PWM_TRANSFER = 0;

	// Drive Train Encoders
	public static final int DRIVE_TRAIN_LEFT_ENCODER_A = 0;
	public static final int DRIVE_TRAIN_LEFT_ENCODER_B = 1;
	public static final int DRIVE_TRAIN_RIGHT_ENCODER_A = 2;
	public static final int DRIVE_TRAIN_RIGHT_ENCODER_B = 3;

}