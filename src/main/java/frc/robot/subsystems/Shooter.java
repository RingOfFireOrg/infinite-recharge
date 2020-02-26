package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import frc.robot.subsystems.Climber.ClimberState;

//Prototype code authors: (Duncan, Bella, Jeremy)?

public class Shooter extends InternalSubsystem {

    CANSparkMax feederMotor;
    CANSparkMax shooterMotor;

    double baseShooterSpeed = 0.56;

    public enum shooterStates {
        OFF, BASE_SPEED
    }

    shooterStates state;

    public Shooter() {
        feederMotor = new CANSparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER_OUTPUT_MOTOR, MotorType.kBrushless);
        state = shooterStates.OFF;
    }

    public void setState(shooterStates state) {
        this.state = state;
    }

    public boolean isSpunUp() {
        // Returns true if shooter RPM is in ideal speed range
        return true;
    }

    public int getState() {
        // Returns current shooter state
        return 1;
    }

    public void setShooterSpeed(double speed) {
        baseShooterSpeed = speed;
    }

    public void teleopControl() {
        // Takes driver input and sets states
        if (super.controlSystem.getManipulatorRightTrigger() > 0.05) {
            setState(shooterStates.BASE_SPEED);
        } else {
            setState(shooterStates.OFF);
        }
        baseShooterSpeed = super.controlSystem.getManipulatorRightTrigger();
    }

    public void periodic() {
        //this method will be run every code loop
        if (state == shooterStates.BASE_SPEED) {
            shooterMotor.set(baseShooterSpeed);
        } else {
            shooterMotor.set(0);
        }

    }
}
