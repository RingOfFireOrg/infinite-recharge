package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber.ClimberState;

//Prototype code authors: (Duncan, Bella, Jeremy)?

public class Shooter extends InternalSubsystem {

    CANSparkMax feederMotor;
    CANSparkMax shooterMotor;

    double baseShooterSpeed = 0.56;
    double baseLowerShooterSpeed = 1;

    public enum shooterStates {
        OFF, BASE_SPEED
    }

    shooterStates upperState;
    shooterStates lowerState;

    public Shooter() {
        feederMotor = new CANSparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER_OUTPUT_MOTOR, MotorType.kBrushless);
        upperState = shooterStates.OFF;
        lowerState = shooterStates.OFF;
        feederMotor.set(0);
    }

    public void setState(shooterStates state) {
        this.upperState = state;
    }

    public void setLowerShooterState(shooterStates state) {
        this.lowerState = state;
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

    public void setLowerShooterSpeed(double speed){
        baseLowerShooterSpeed = speed;
    }

    public void teleopControl() {
        //Takes driver input and sets states
        if (super.controlSystem.getManipulatorRightTrigger() > 0.05) {
            setState(shooterStates.BASE_SPEED);
        } else {
            setState(shooterStates.OFF);
        }
        if (Math.abs(super.controlSystem.getManipulatorLeftTrigger()) > 0.05) {
            setLowerShooterState(shooterStates.BASE_SPEED);
        } else {
            setLowerShooterState(shooterStates.OFF);
        }
        baseShooterSpeed = super.controlSystem.getManipulatorRightTrigger() * 1;
        baseLowerShooterSpeed = super.controlSystem.getManipulatorLeftTrigger() * 1;
    }

    public void periodic() {
        //this method will be run every code loop
        if (upperState == shooterStates.BASE_SPEED) {
            shooterMotor.set(super.controlSystem.rightDriveStick.getRawAxis(3));
            SmartDashboard.putNumber("slider", 0.5 + ((super.controlSystem.rightDriveStick.getRawAxis(3) + 1) / 4));
        } else {
            shooterMotor.set(0);
        }
        SmartDashboard.putNumber("slider", 0.5 + ((super.controlSystem.rightDriveStick.getRawAxis(3) + 1) / 4));

        if (lowerState == shooterStates.BASE_SPEED) {
            feederMotor.set(baseLowerShooterSpeed);
        } else {
            feederMotor.set(0);
        }

    }
}
