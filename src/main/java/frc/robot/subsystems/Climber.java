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
        IDLE, WINDING_UP
    }

    private TalonSRX extensionMotor, climbMotor;
    private PWMVictorSPX traverseMotor;
    private ClimberState state;
    private WinchState winchState;

    private double traverseSpeed = 0;

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

    public void teleopControl() {
        if (super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER) > 0.1) {
            setState(ClimberState.EXTENDING);
        } else if (super.controlSystem.manipulatorGamepadEndgame.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER) > 0.1) {
            setState(ClimberState.RETRACTING);
        } else {
            setState(ClimberState.IDLE);
        }

        if (super.controlSystem.manipulatorGamepadEndgame.getAButton()) {
            winchState = winchState.WINDING_UP;
        } else {
            winchState = winchState.IDLE;
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
            extensionMotor.set(ControlMode.PercentOutput, 1);
        } else if (state == ClimberState.EXTENDING) {
            // climbMotor.set(ControlMode.PercentOutput, 1);
            extensionMotor.set(ControlMode.PercentOutput, -1);
        }

        if (winchState == WinchState.WINDING_UP) {
            climbMotor.set(ControlMode.PercentOutput, 1);
        } else if (state == ClimberState.EXTENDING) {
            climbMotor.set(ControlMode.PercentOutput, -1);
        } else {
            climbMotor.set(ControlMode.PercentOutput, 0);
        }

        traverseMotor.set(traverseSpeed);
    }
}
