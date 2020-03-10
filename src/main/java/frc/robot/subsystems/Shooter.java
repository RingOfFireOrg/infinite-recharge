package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.PID;


public class Shooter extends InternalSubsystem {

    CANSparkMax feederMotor;
    CANSparkMax shooterMotor;
    CANEncoder shooterEncoder;
    CANEncoder feederEncoder;

    public final static double BASE_SHOOTER_RPM = 3300;
    public final static double FULL_SPEED_RPM = RobotMap.NEO_TOP_RPM;

    double baseLowerShooterSpeed = 1;

    PID shooterControl;

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
        
        shooterEncoder = shooterMotor.getEncoder();
        feederEncoder = feederMotor.getEncoder();

        shooterControl = new PID(1.8, 0.006, 0);
        shooterControl.setOutputRange(-1000, 1000);

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
        //BASE_SHOOTER_SPEED = speed;
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
        //baseShooterSpeed = super.controlSystem.getManipulatorRightTrigger() * 1;
        baseLowerShooterSpeed = super.controlSystem.getManipulatorLeftTrigger() * 1;
    }

    public void periodic() {
        //this method will be run every code loop
        if (upperState == shooterStates.BASE_SPEED) {
            shooterControl.setError(BASE_SHOOTER_RPM - shooterEncoder.getVelocity());
            shooterControl.update();
            shooterMotor.set((BASE_SHOOTER_RPM + shooterControl.getOutput()) / RobotMap.NEO_TOP_RPM);
            // shooterControl.setError(FULL_SPEED_RPM - shooterEncoder.getVelocity());
            // shooterControl.update();
            // shooterMotor.set((FULL_SPEED_RPM + shooterControl.getOutput()) / RobotMap.NEO_TOP_RPM);
        } else {
            shooterMotor.set(0);
        }

        if (lowerState == shooterStates.BASE_SPEED) {
            feederMotor.set(baseLowerShooterSpeed);
        } else {
            feederMotor.set(0);
        }
        SmartDashboard.putNumber("shooterSpeed", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("pid output", shooterControl.getOutput());

    }
}
