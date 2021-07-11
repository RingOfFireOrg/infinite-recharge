// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.PWMVictorSPX;
// import frc.robot.RobotMap;

// //Prototype code authors: (Bella, Isha, Kyndall)?

// public class ControlPanel extends InternalSubsystem {

//     public enum VerticalControlPanelStates {
//         EXTENDING, RETRACTING, IDLE
//     }

//     public enum SpinnerStates {
//         LEFT, RIGHT, AUTO, IDLE
//     }

//     private double spinnerSpeed;
//     private VerticalControlPanelStates actuatorState;
//     private SpinnerStates spinnerState;

//     private PWMVictorSPX actuatorMotor, spinnerMotor;

//     public ControlPanel () {
//         spinnerSpeed = 0;
//         actuatorState = VerticalControlPanelStates.IDLE;
//         spinnerState = SpinnerStates.IDLE;
//         actuatorMotor = new PWMVictorSPX(RobotMap.CONTROL_PANEL_ACTUATOR);
//         spinnerMotor = new PWMVictorSPX(RobotMap.CONTROL_PANEL_SPIN);
//     }

//     public void setVerticalState(VerticalControlPanelStates state) {
//         this.actuatorState = state;
//     }

//     public void setSpinnerState(SpinnerStates state) {
//         this.spinnerState = state;
//     }

//     public void teleopControl() {
//         if (super.controlSystem.getManipulatorLeftY() > 0.5) {
//             setVerticalState(VerticalControlPanelStates.RETRACTING);
//         } else if (super.controlSystem.getManipulatorLeftY() < -0.5) {
//             setVerticalState(VerticalControlPanelStates.EXTENDING);
//         } else {
//             setVerticalState(VerticalControlPanelStates.IDLE);
//         }

//         if (super.controlSystem.manualSpinner.get()) {
//             setSpinnerState(SpinnerStates.LEFT);
//         } else {
//             setSpinnerState(SpinnerStates.IDLE);
//         }
//     }
    
//     @Override
//     public void periodic() {
//         if (actuatorState == VerticalControlPanelStates.EXTENDING) {
//             actuatorMotor.set(1);
//         } else if (actuatorState == VerticalControlPanelStates.RETRACTING) {
//             actuatorMotor.set(-1);
//         } else {
//             actuatorMotor.set(0);
//         }

//         if (spinnerState == SpinnerStates.LEFT) {
//             spinnerMotor.set(-1);
//         } else if (spinnerState == SpinnerStates.RIGHT) {
//             spinnerMotor.set(1);
//         } else {
//             spinnerMotor.set(0);
//         }
//     }
// }