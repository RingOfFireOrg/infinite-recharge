package frc.robot;


import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


public class ControlSystems {

    public Joystick leftDriveStick, rightDriveStick;
    public XboxController manipulatorGamepad;
    public JoystickButton climberExtend, climberRetract, intakeForward, intakeReverse, positionControl, rotationControl;


    private static ControlSystems controlSystems;

    protected ControlSystems() {
        leftDriveStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
        rightDriveStick= new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
        manipulatorGamepad = new XboxController(RobotMap.GAMEPAD_MANIPULATOR);

        climberExtend = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_LEFT_BUMPER);
        climberRetract = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_RIGHT_BUMPER);
        intakeForward = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_A_BUTTON);
        intakeReverse = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_B_BUTTON);
        positionControl = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_X_BUTTON);
        rotationControl = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_Y_BUTTON); 

    //     public static final int MANIPULATOR_LEFT_BUMPER = 5;
	// public static final int MANIPULATOR_RIGHT_BUMPER = 6;
	// public static final int MANIPULATOR_A_BUTTON = 1;
	// public static final int MANIPULATOR_B_BUTTON = 2;
	// public static final int MANIPULATOR_X_BUTTON = 3;
	// public static final int MANIPULATOR_Y_BUTTON = 4;
    }

    public static ControlSystems getInstance() {
        if (controlSystems == null) {
            controlSystems = new ControlSystems();
        }
        return controlSystems;
    }

    public double getManipulatorLeftY() {
        return manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_LEFT_JOYSTICK_Y);
    }

    public double getManipulatorRightY() {
        return manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_Y);
    }

    public double getManipulatorLeftTrigger() {
        return manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER);
    }

    public double getManipulatorRightTrigger() {
        return manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER);
    }

    // public static final int MANIPULATOR_RIGHT_TRIGGER = 3;
	// public static final int MANIPULATOR_LEFT_TRIGGER = 2;
	// public static final int MANIPULATOR_LEFT_JOYSTICK_Y = 1;
	// public static final int MANIPULATOR_RIGHT_JOYSTICK_Y = 5;

}