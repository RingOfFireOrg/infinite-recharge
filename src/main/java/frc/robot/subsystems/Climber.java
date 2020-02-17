package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;

import frc.robot.RobotMap;

public class Climber extends InternalSubsystem {

    enum State 
    { 
        IDLE,
        EXTENDING, RETRACTING; 
    }

    private PWMVictorSPX climberUpMotor;
    private PWMVictorSPX winchMotor;
    private PWMVictorSPX traverseMotor;
    private State state;
    
    
    
    public Climber() {
        climberUpMotor = new PWMVictorSPX(RobotMap.CLIMBER_UP);
        winchMotor = new PWMVictorSPX(RobotMap.WINCH);
        state = State.IDLE;
    }

    public void setState(State state){
        this.state = state;
        
    }

    public boolean setTraverseSpeed(double speed) {
        return true;
    }

    public void periodic() {
        //this method will be run every code loop
    }
}
