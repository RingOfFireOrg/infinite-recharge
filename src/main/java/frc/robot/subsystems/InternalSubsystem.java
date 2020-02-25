package frc.robot.subsystems;

import frc.robot.ControlSystems;

//all subsytems should extend this class
abstract public class InternalSubsystem {

    //a control system for all instances to be able to access
    protected ControlSystems controlSystem = ControlSystems.getInstance();

    //should control the teleop state setting of the subsystem -- get input from control system and set states
    abstract public void teleopControl();

    //will be run by the robotContainer every loop of the code
    abstract public void periodic();
}