package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PWMVictorSPX;

public class SRXExample {

    TalonSRX motor; 
    double speedMultiplier = 1;

    public SRXExample(int canbusadress) {

        motor = new TalonSRX(canbusadress);
    }

    public void spin (double speedVector) {

        motor.set(ControlMode.PercentOutput, speedVector * speedMultiplier);

    }
    public void stop (){

        motor.set(ControlMode.PercentOutput, 0);
    }
    public void setSensitivity (double sensitivity){

        speedMultiplier = sensitivity;
    } 
    public double getSensitivity () {

        return speedMultiplier;
    }
}