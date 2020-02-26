package frc.robot.subsystems;

import frc.robot.RobotMap;

//Prototype code authors: (Duncan, Aaron)?

public class Indexer extends InternalSubsystem {

    
    public Indexer() {

    }

    public boolean setForward() {
        // Moves balls to shooter
        return true;
    }

    public boolean setBack() {
        // Moves balls toward intake
        return true;
    }

    public boolean setOff() {
        // Sets indexer to 0
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
        //this method will be run every code loop
    }
}
