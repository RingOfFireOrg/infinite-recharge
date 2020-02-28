package frc.robot;  

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class ControlSystems {

    public Joystick leftDriveStick, rightDriveStick;
    public XboxController manipulatorGamepadTeleop, manipulatorGamepadEndgame;
    public JoystickButton indexerIn, indexerOut, intakeForward, intakeReverse, positionControl, manualSpinner, switchCameraViewManipulator, switchCameraViewDriver

    public enum directions {
        EAST, NORTHEAST, NORTH, NORTHWEST, WEST, SOUTHWEST, SOUTH, SOUTHEAST, NEUTRAL
    }

    private static ControlSystems controlSystems;

    protected ControlSystems() {
        leftDriveStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
        rightDriveStick = new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
        manipulatorGamepadTeleop = new XboxController(RobotMap.GAMEPAD_MANIPULATOR);
        manipulatorGamepadEndgame = new XboxController(RobotMap.GAMEPAD_ENDGAME);

        indexerOut = new JoystickButton(manipulatorGamepadTeleop, RobotMap.MANIPULATOR_LEFT_BUMPER);
        indexerIn = new JoystickButton(manipulatorGamepadTeleop, RobotMap.MANIPULATOR_RIGHT_BUMPER);
        intakeForward = new JoystickButton(manipulatorGamepadTeleop, RobotMap.MANIPULATOR_A_BUTTON);
        intakeReverse = new JoystickButton(manipulatorGamepadTeleop, RobotMap.MANIPULATOR_B_BUTTON);
        positionControl = new JoystickButton(manipulatorGamepadTeleop, RobotMap.MANIPULATOR_X_BUTTON);
        manualSpinner = new JoystickButton(manipulatorGamepadTeleop, RobotMap.MANIPULATOR_Y_BUTTON);
        switchCameraViewManipulator = new JoystickButton(manipulatorGamepadTeleop, RobotMap.MANIPULATOR_START_BUTTON);
        switchCameraViewDriver = new JoystickButton(leftDriveStick, RobotMap.DRIVER_TRIGGER);

    }

    public static ControlSystems getInstance() {
        if (controlSystems == null) {
            controlSystems = new ControlSystems();
        }
        return controlSystems;
    }

    public double getManipulatorLeftY() {
        return manipulatorGamepadTeleop.getRawAxis(RobotMap.MANIPULATOR_LEFT_JOYSTICK_Y);
    }

    public double getManipulatorRightY() {
        return manipulatorGamepadTeleop.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_Y);
    }

    public double getManipulatorLeftTrigger() {
        return manipulatorGamepadTeleop.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER);
    }

    public double getManipulatorRightTrigger() {
        return manipulatorGamepadTeleop.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER);
    }

    public directions getDPAD() {
        if (manipulatorGamepadTeleop.getPOV() == 0) {
            return directions.NORTH;
        } else if (manipulatorGamepadTeleop.getPOV() == 45) {
            return directions.NORTHEAST;
        } else if (manipulatorGamepadTeleop.getPOV() == 90) {
            return directions.EAST;
        } else if (manipulatorGamepadTeleop.getPOV() == 135) {
            return directions.SOUTHEAST;
        } else if (manipulatorGamepadTeleop.getPOV() == 180) {
            return directions.SOUTH;
        } else if (manipulatorGamepadTeleop.getPOV() == 225) {
            return directions.SOUTHWEST;
        } else if (manipulatorGamepadTeleop.getPOV() == 270) {
            return directions.WEST;
        } else if (manipulatorGamepadTeleop.getPOV() == 315) {
            return directions.NORTHWEST;
        } else {
            return directions.NEUTRAL;
        }
    }

    public directions getEndgameDPAD() {
        if (manipulatorGamepadEndgame.getPOV() == 0) {
            return directions.NORTH;
        } else if (manipulatorGamepadEndgame.getPOV() == 45) {
            return directions.NORTHEAST;
        } else if (manipulatorGamepadEndgame.getPOV() == 90) {
            return directions.EAST;
        } else if (manipulatorGamepadEndgame.getPOV() == 135) {
            return directions.SOUTHEAST;
        } else if (manipulatorGamepadEndgame.getPOV() == 180) {
            return directions.SOUTH;
        } else if (manipulatorGamepadEndgame.getPOV() == 225) {
            return directions.SOUTHWEST;
        } else if (manipulatorGamepadEndgame.getPOV() == 270) {
            return directions.WEST;
        } else if (manipulatorGamepadEndgame.getPOV() == 315) {
            return directions.NORTHWEST;
        } else {
            return directions.NEUTRAL;
        }
    }

}