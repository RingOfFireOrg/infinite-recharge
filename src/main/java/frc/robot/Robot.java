/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Robot extends TimedRobot {
    CimTank tankDrive;
    Joystick rightstick;
    Joystick leftstick;
    Joystick manipulatorStick;
    JoystickButton intakeButton;
    CANSparkMax intakeMotor;

    @Override
    public void robotInit() {
        tankDrive = new CimTank();
        Joystick rightstick = new Joystick(0);
        Joystick leftstick = new Joystick(1);
        Joystick manipulatorStick = new Joystick(2);
        JoystickButton intakeButton = new JoystickButton(manipulatorStick, 1);

        CANSparkMax intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        double rightSpeed = rightstick.getY();
        double leftSpeed = leftstick.getY();
        boolean intakeButtonPressed = intakeButton.get();

        tankDrive.drive(leftSpeed, rightSpeed, true, 1);

        if (intakeButtonPressed) {
            intakeMotor.set(0.4);
        } else {
            intakeMotor.set(0);
        }
    }

    @Override
    public void testPeriodic() {
    }
}
