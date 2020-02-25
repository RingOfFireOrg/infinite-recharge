package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ControlSystems {

    public Joystick leftDriveStick, rightDriveStick;
    public XboxController manipulatorGamepad;
    private static ControlSystems controlSystems;

    protected ControlSystems() {
        leftDriveStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
        rightDriveStick= new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
        manipulatorGamepad = new XboxController(RobotMap.GAMEPAD_MANIPULATOR);
    }

    public static ControlSystems getInstance() {
        if (controlSystems == null) {
            controlSystems = new ControlSystems();
        }
        return controlSystems;
    }
}