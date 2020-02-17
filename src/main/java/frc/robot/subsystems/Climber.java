package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;

import frc.robot.RobotMap;

public class Climber extends InternalSubsystem {

    public PWMVictorSPX climberUpMotor;
    public PWMVictorSPX winchMotor;

    public Climber() {
        climberUpMotor = new PWMVictorSPX(RobotMap.CLIMBER_UP);
        winchMotor = new PWMVictorSPX(RobotMap.WINCH);

    }

    public boolean setExtend() {
        return true;
    }

    public boolean setRetract() {
        return true;
    }

    public boolean setTraverseSpeed() {
        return true;
    }

    public void periodic() {
        //this method will be run every code loop
    }
}
