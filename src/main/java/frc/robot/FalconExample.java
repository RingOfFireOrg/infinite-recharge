package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class FalconExample implements ExampleMotor {

    TalonFX motor; 
    double speedMultiplier = 1;

    public FalconExample (int canbusadress) {

        motor = new TalonFX(canbusadress);
    }

    public void spin (double speedVector) {

        motor.set(TalonFXControlMode.PercentOutput, speedVector * speedMultiplier);

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







