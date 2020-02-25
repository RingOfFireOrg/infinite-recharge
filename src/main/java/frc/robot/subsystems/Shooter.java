package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

//Prototype code authors: (Duncan, Bella, Jeremy)?

public class Shooter extends InternalSubsystem {

    CANSparkMax feederMotor;
    CANSparkMax shooterMotor;

    double goalShooterSpeed;

    public Shooter() {
        feederMotor = new CANSparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER_OUTPUT_MOTOR, MotorType.kBrushless);
    }

    public boolean setShoot() {
        // Set to shooting speed
        return true;
    }

    public boolean setSlow() {
        // Used for testing or removing balls from shooter safely
        return true;
    }

    public boolean isSpunUp() {
        // Returns true if shooter RPM is in ideal speed range
        return true;
    }

    public int getState() {
        // Returns current shooter state
        return 1;
    }

    public void teleopControl() {
        // Takes driver input and sets states
    }

    public void periodic() {
        //this method will be run every code loop
    }
}
