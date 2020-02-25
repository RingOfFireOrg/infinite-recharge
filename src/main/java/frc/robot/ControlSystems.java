package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


public class ControlSystems {

    public Joystick leftDriveStick, rightDriveStick;
    public GenericHID manipulatorGamepad;
    private static ControlSystems controlSystems;

    protected ControlSystems() {
        leftDriveStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
        rightDriveStick= new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
       // manipulatorGamepad = new (RobotMap.GAMEPAD_MANIPULATOR);

    }

    public static ControlSystems getInstance() {
        if (controlSystems == null) {
            controlSystems = new ControlSystems();
        }
        return controlSystems;
    }
}