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


}