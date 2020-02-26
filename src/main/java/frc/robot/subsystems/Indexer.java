package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.VictorSP;

//Prototype code authors: (Duncan, Aaron, Gray)?

public class Indexer extends InternalSubsystem {
    VictorSP indexMotor;

    enum IndexerState {
        IDLE, FORWARD, BACKWARD
    }

    IndexerState state;

    public Indexer() {
        indexMotor = new VictorSP(RobotMap.INDEXER_MOTOR);
        state = IndexerState.IDLE;
    }

    public boolean setForward() {
        state = IndexerState.FORWARD;
        return true;
    }

    public boolean setBackward() {
        state = IndexerState.BACKWARD;
        return true;
    }

    public boolean setOff() {
        state = IndexerState.IDLE;
        return true;
    }

    public void setState(IndexerState desiredState) {  // Never used; do we need this?
        this.state = desiredState;
    }

    public int getState() {
        // Returns the current state of the indexer
        return 1;
    }

    public void teleopControl() {
        // Takes driver input and sets states
    }

    public void periodic() {
        //this method will be run every code loop
        if (state == IndexerState.FORWARD) {
            indexMotor.set(1);
        } else if (state == IndexerState.BACKWARD){
            indexMotor.set(-1);        
        } else if (state == IndexerState.IDLE) {
            indexMotor.set(0);
        }
    }
}
