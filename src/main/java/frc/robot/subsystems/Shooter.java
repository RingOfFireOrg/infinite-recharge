package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

public class Shooter extends InternalSubsystem {

    CANSparkMax feederMotor;
    CANSparkMax shooterMotor;

    double goalShooterSpeed;

    public Shooter() {
        feederMotor = new CANSparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER_OUTPUT_MOTOR, MotorType.kBrushless);
    }

    public boolean setShoot() {

        return true;
    }

    public boolean setSlow() {

        return true;
    }

    public boolean isSpunUp() {
        return true;
    }

    public int getState() {

        return 1;
    }

    public void teleopControl() {
        
    }

    public void periodic() {
        //this method will be run every code loop
    }
}
