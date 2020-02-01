/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

    Joystick rightstick = new Joystick(0);
    Joystick leftstick = new Joystick(1);
    Joystick manipulatorStick = new Joystick(2);
    JoystickButton intakeButton = new JoystickButton(manipulatorStick, 1);

    CANSparkMax intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);

    @Override
    public void robotInit() {
        tankDrive = new CimTank();

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
        double rightSpeed = -rightstick.getY();
        double leftSpeed = -leftstick.getY();
        double intakeSpeed = manipulatorStick.getY();
        boolean intakeButtonPressed = intakeButton.get();

        tankDrive.drive(leftSpeed, rightSpeed, false, 1);

        intakeMotor.set(-intakeSpeed);
    }

    @Override
    public void testPeriodic() {
    }
}
