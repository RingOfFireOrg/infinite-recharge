package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;

import frc.robot.RobotMap;

public class Climber extends InternalSubsystem {

    enum ClimberState { 
        IDLE,EXTENDING, RETRACTING; 
    }

    private PWMVictorSPX climberUpMotor;
    private PWMVictorSPX winchMotor;
    private PWMVictorSPX traverseMotor;
    private ClimberState state;
    
    private double traverseSpeed = 0;
    
    public Climber() {
        climberUpMotor = new PWMVictorSPX(RobotMap.CLIMBER_EXTENSION);
        winchMotor = new PWMVictorSPX(RobotMap.CLIMBER_WINCH);
        traverseMotor = new PWMVictorSPX(RobotMap.CLIMBER_TRAVERSE);
        state = ClimberState.IDLE;
    }

    public void setState(ClimberState state){
        this.state = state;
    }

    public boolean setTraverseSpeed(double speed) {
        traverseSpeed = speed;
        return true;
    }

    public void periodic() {
        //this method will be run every code loop
        if (state == ClimberState.IDLE) {
            winchMotor.set(0);
            climberUpMotor.set(0);
        } else if (state == ClimberState.RETRACTING) {
            winchMotor.set(-1);
            climberUpMotor.set(1);
        } else if (state == ClimberState.EXTENDING) {
            winchMotor.set(1);
            climberUpMotor.set(-1);
        }

        traverseMotor.set(traverseSpeed);
    }
}
