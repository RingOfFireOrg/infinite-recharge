package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class GamepadControl implements GenericMotorControl {

    XboxController aController;
    ExampleMotor aMotor;
    double speed = 0;

    public GamepadControl (XboxController myController, ExampleMotor myMotor) {

        aController = myController;
        aMotor = myMotor;
    }

    @Override
    public void readInputs() {

        boolean buttonA = aController.getRawButton(2);
        boolean buttonB = aController.getRawButton(3);

        if (buttonA) {
            speed = 1;
        } else if (buttonB) {
            speed = -1;
        } else {
            speed = 0;
        }
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