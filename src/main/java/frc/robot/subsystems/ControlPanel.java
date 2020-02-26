package frc.robot.subsystems;

//Prototype code authors: (Bella, Isha, Kyndall)?

public class ControlPanel extends InternalSubsystem {

    public enum VerticalControlPanelStates {
        EXTENDING, RETRACTING, IDLE
    }

    public enum SpinnerStates {
        LEFT, RIGHT, AUTO, IDLE
    }

    private double spinnerSpeed;

    public ControlPanel () {
        spinnerSpeed = 0;
    }

    public void setVerticalState(VerticalControlPanelStates state) {
 //finish
    }

    public void teleopControl() {
        
    }
    
    @Override
    public void periodic() {
        
    }
}