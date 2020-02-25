package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PWMVictorSPX;

import frc.robot.RobotMap;

//Prototype code authors: () Rachel

//Prototype code authors: 
public class Climber extends InternalSubsystem {

    enum ClimberState { 
        IDLE,EXTENDING, RETRACTING; 
    }

    private TalonSRX extensionMotor, climbMotor, traverseMotor;
    private ClimberState state;
    
    private double traverseSpeed = 0;
    
    public Climber() {
        extensionMotor = new TalonSRX(RobotMap.CLIMBER_EXTENSION);
        climbMotor = new TalonSRX(RobotMap.CLIMBER_WINCH);
        traverseMotor = new TalonSRX(RobotMap.CLIMBER_TRAVERSE);
        state = ClimberState.IDLE;
    }

    public void setState(ClimberState state){
        this.state = state;
    }

    public boolean setTraverseSpeed(double speed) {
        traverseSpeed = speed;
        return true;
    }

    public void teleopControl() {
        
    }

    public void periodic() {
        //this method will be run every code loop
        if (state == ClimberState.IDLE) {
            climbMotor.set(ControlMode.PercentOutput, 0);
            extensionMotor.set(ControlMode.PercentOutput, 0);
        } else if (state == ClimberState.RETRACTING) {
            climbMotor.set(ControlMode.PercentOutput,-1);
            extensionMotor.set(ControlMode.PercentOutput, 1);
        } else if (state == ClimberState.EXTENDING) {
            climbMotor.set(ControlMode.PercentOutput, 1);
            extensionMotor.set(ControlMode.PercentOutput, -1);
        }

        traverseMotor.set(ControlMode.PercentOutput, traverseSpeed);
    }
}
