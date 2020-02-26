package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.RobotMap;

//Prototype code authors: (Duncan, Aaron, Gray)?

public class Indexer extends InternalSubsystem {
    enum IndexerState {
        IDLE, FORWARD, BACKWARD
    }


    IndexerState state;
    PWMVictorSPX indexMotor = new PWMVictorSPX(0);
    
    public Indexer() {
        state = IndexerState.IDLE;
    }

    public void setState(IndexerState state) {
        this.state = state;
    }

    public int getState() {
        // Returns the current state of the indexer
        return 1;
    }

    public void teleopControl() {
        // Takes driver input and sets states
        if (super.controlSystem.indexerIn.get() == true) {
            setState(IndexerState.FORWARD);
        } else if (super.controlSystem.indexerOut.get() == true) {
            setState(IndexerState.BACKWARD);
        } else {
            setState(IndexerState.IDLE);
        }
    }

    public void periodic() {
        if (state == IndexerState.FORWARD) {
            indexMotor.set(0.75);
        } else if (state == IndexerState.BACKWARD){
            indexMotor.set(-0.75);        
        } else if (state == IndexerState.IDLE){
            indexMotor.set(0);
        }
        
        //this method will be run every code loop
    }
}
