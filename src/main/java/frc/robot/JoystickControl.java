package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class JoystickControl implements GenericMotorControl {

    Joystick aJoystick;
    ExampleMotor aMotor;
    double speed = 0;

    public JoystickControl (Joystick myJoystick, ExampleMotor myMotor) {

        aJoystick = myJoystick;
        aMotor = myMotor;
    }

    @Override
    public void readInputs() {

        speed = aJoystick.getY();

    }

    @Override
    public void activateMotorSpeed() {

        aMotor.spin (speed);
    }

    @Override
    public double getMotorSpeed() {

        return speed;
    }
}