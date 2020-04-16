package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

class ButtonControl {

    ExampleMotor aMotor;

    public ButtonControl (ExampleMotor myMotor) {
        aMotor = myMotor;
    }

    public void fullForward () {
    
        aMotor.spin(1);
    }
    public void fullBackward () {

        aMotor.spin(-1);
    }
    public void fullStop () {

        aMotor.stop();
    }
}