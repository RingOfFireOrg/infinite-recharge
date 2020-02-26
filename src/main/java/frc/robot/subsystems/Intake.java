package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends InternalSubsystem {

    private CANSparkMax intakeMotor;

    public enum IntakeStates {
        IN, OUT, IDLE
    }
    
    IntakeStates state;

    public Intake() {
        intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
        state = IntakeStates.IDLE;
    }

    public boolean setState(IntakeStates desiredState) {
        this.state = desiredState;
        return true;
    }

    public IntakeStates getState() {
        return state;
    }

    public void teleopControl() {
        if (super.controlSystem.intakeForward.get()) {
            state = IntakeStates.IN;
        } else if (super.controlSystem.intakeReverse.get()) {
            state = IntakeStates.OUT;
        } else {
            state = IntakeStates.IDLE;
        }
    } 

    public void periodic() {
        //this method will be run every code loop during teleop and autonomous
        if (state == IntakeStates.IDLE) {
            intakeMotor.set(0);
        } else if (state == IntakeStates.OUT) {
            intakeMotor.set(-0.6);
        } else if (state == IntakeStates.IN) {
            intakeMotor.set(0.6);
        }
    }
}
