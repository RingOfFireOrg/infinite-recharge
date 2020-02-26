package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.RobotMap;

//Prototype code authors: (Duncan, Aaron, Gray)?

public class Indexer extends InternalSubsystem {
    enum IndexerState {
        IDLE, FORWARD, BACKWARD
    }


    IndexerState state;
    VictorSP indexMotor = new VictorSP(0);
    
    public Indexer() {
        state = IndexerState.IDLE;
    }

    public boolean setForward() {
        // Moves balls to shooter
        state = IndexerState.FORWARD;
        return true;
    }

    public boolean setBack() {
        // Moves balls toward intake
        state = IndexerState.BACKWARD;
        return true;
    }

    public boolean setOff() {
        // Sets indexer to 0
        state = IndexerState.IDLE;
        return true;
    }

    public int getState() {
        // Returns the current state of the indexer
        return 1;
    }

    public void teleopControl() {
        // Takes driver input and sets states
    }

    public void periodic() {
        if (state == IndexerState.FORWARD) {
            indexMotor.set(1);
        } else if (state == IndexerState.BACKWARD){
            indexMotor.set(-1);        
        } else if (state == IndexerState.IDLE){
                indexMotor.set(0);
        }
        
        //this method will be run every code loop
    }
}
