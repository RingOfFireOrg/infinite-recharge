package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import frc.robot.RobotMap;
import frc.robot.ControlSystems.directions;

//Prototype code authors: () Rachel

//Prototype code authors: 
public class Climber extends InternalSubsystem {

    enum ClimberState {
        IDLE, EXTENDING, RETRACTING;
    }

    enum WinchState {
        IDLE, WINDING_UP, WINDING_DOWN
    }

    private TalonSRX extensionMotor, climbMotor;
    private PWMVictorSPX traverseMotor;
    private ClimberState state;
    private WinchState winchState;

    private double traverseSpeed = 0;
    private double liftSpeed = 0;
    private double winchSpeed = 0;

    public Climber() {
        extensionMotor = new TalonSRX(RobotMap.CLIMBER_EXTENSION);
        climbMotor = new TalonSRX(RobotMap.CLIMBER_WINCH);
        traverseMotor = new PWMVictorSPX(RobotMap.CLIMBER_TRAVERSE);
        state = ClimberState.IDLE;
        winchState = WinchState.IDLE;
        extensionMotor.set(ControlMode.PercentOutput, 0);
        climbMotor.set(ControlMode.PercentOutput, 0);
        traverseMotor.set(0);
    }

    public void setState(ClimberState state) {
        this.state = state;
    }

    public boolean setTraverseSpeed(double speed) {
        traverseSpeed = speed;
        return true;
    }

    public void setLift(double liftSpeed) {
        this.liftSpeed = liftSpeed;
        if (liftSpeed > 0) {
            state = ClimberState.EXTENDING;
        } else if (liftSpeed < 0) {
            state = ClimberState.RETRACTING;
        } else {
            state = ClimberState.IDLE;
        }
    } 

    public void setWinch(double winchSpeed) {
        this.winchSpeed = winchSpeed;
        if (winchSpeed > 0) {
            winchState = winchState.WINDING_UP;
        } else if (winchSpeed < 0) {
            winchState = winchState.WINDING_DOWN;
        } else {
            winchState = winchState.IDLE;
        }
    }

    public void teleopControl() {
        // if (super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER) > 0.1) {
        //     setState(ClimberState.EXTENDING);
        // } else if (super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER) > 0.1) {
        //     setState(ClimberState.RETRACTING);
        // } else {
        //     setState(ClimberState.IDLE);
        // }

        // if (super.controlSystem.manipulatorGamepadEndgame.getAButton()) {
        //     winchState = winchState.WINDING_UP;
        // } else {
        //     winchState = winchState.IDLE;
        // }

        if (Math.abs(super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_Y)) > 0.1) {
            setLift(-super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_Y));
        } else {
            setLift(0);
        }

        if (Math.abs(super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_LEFT_JOYSTICK_Y)) > 0.1) {
            setWinch(-super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_LEFT_JOYSTICK_Y));
        } else {
            setWinch(0);
        }

        if (super.controlSystem.getEndgameDPAD() == directions.WEST) {
            setTraverseSpeed(-1);
        } else if (super.controlSystem.getEndgameDPAD() == directions.EAST) {
            setTraverseSpeed(1);
        } else {
            setTraverseSpeed(0);
        }
    }

    public void periodic() {
        // this method will be run every code loop
        if (state == ClimberState.IDLE) {
            // climbMotor.set(ControlMode.PercentOutput, 0);
            extensionMotor.set(ControlMode.PercentOutput, 0);
        } else if (state == ClimberState.RETRACTING) {
            // climbMotor.set(ControlMode.PercentOutput,-1);
            extensionMotor.set(ControlMode.PercentOutput, liftSpeed);
        } else if (state == ClimberState.EXTENDING) {
            // climbMotor.set(ControlMode.PercentOutput, 1);
            extensionMotor.set(ControlMode.PercentOutput, liftSpeed);
        }

        if (winchState == WinchState.WINDING_UP) {
            climbMotor.set(ControlMode.PercentOutput, winchSpeed);
        } else if (state == ClimberState.EXTENDING || winchState == winchState.WINDING_DOWN) {
            climbMotor.set(ControlMode.PercentOutput, winchSpeed);
        } else {
            climbMotor.set(ControlMode.PercentOutput, 0);
        }

        traverseMotor.set(traverseSpeed);
    }
}
