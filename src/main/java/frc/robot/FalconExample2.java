package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class FalconExample2 {

    TalonFX motor; 
    double speedMultiplier = 1;

    public FalconExample2 (int canbusadress) {

        motor = new TalonFX(canbusadress);
    }

    public void spin (Boolean Forwardbool, Boolean Backwardbool, double doubleVector, Boolean DoubleOrBoolean) {

        int ForwardBack = 0;
        if (Forwardbool) {
        ForwardBack = 1;
        } else if (Backwardbool) {
        ForwardBack = -1;
        } else {
        ForwardBack = 0;
        }

        if (DoubleOrBoolean) {
        motor.set(TalonFXControlMode.PercentOutput, doubleVector * speedMultiplier);
        } else{
        motor.set(TalonFXControlMode.PercentOutput, ForwardBack * speedMultiplier);
        }
    }
    public void stop (){

        motor.set(TalonFXControlMode.PercentOutput, 0);
    }
    public void setSensitivity (double sensitivity){

        speedMultiplier = sensitivity;
    } 
    public double getSensitivity () {

        return speedMultiplier;
    }
}

