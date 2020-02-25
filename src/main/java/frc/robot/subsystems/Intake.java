package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

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

    public boolean setState(IntakeStates state) {
        this.state = state;
        return true;
    }

    public IntakeStates getState() {
        return state;
    }

    public void teleopControl() {
        
    }

    public void periodic() {
        //this method will be run every code loop
        if (state == IntakeStates.IDLE) {
            intakeMotor.set(0);
        } else if (state == IntakeStates.OUT) {
            intakeMotor.set(-1);
        } else if (state == IntakeStates.IN) {
            intakeMotor.set(1);
        }
    }
}
