/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


public class Robot extends TimedRobot {

  Joystick rightstick = new Joystick(0);
  Joystick leftstick = new Joystick(1);
  Joystick manipulatorStick = new Joystick(2);
  JoystickButton visionButton = new JoystickButton(rightstick, 1);

  AHRS ahrs;
  NeoTankDrive neoDrive;
  Shooter shooter;
  Vision vision;
  
  public CANSparkMax frontLeftMotor;
  public CANSparkMax frontRightMotor;
  public CANSparkMax backRightMotor;
  public CANSparkMax backLeftMotor;
  public VictorSP leftShooterMotor;
  public VictorSP rightShooterMotor;

  @Override
  public void robotInit() {
    ahrs = new AHRS(SerialPort.Port.kUSB);
    ahrs.reset();

    neoDrive = new NeoTankDrive();
    shooter = new Shooter();
    vision = new Vision();
    
    frontLeftMotor = new CANSparkMax(RobotMap.NEO_FRONT_LEFT, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(RobotMap.NEO_FRONT_RIGHT, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(RobotMap.NEO_BACK_RIGHT, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(RobotMap.NEO_BACK_LEFT, MotorType.kBrushless);

    vision.initVision();
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
    double manipulatorStickSpeed = manipulatorStick.getY();
    boolean visionButtonPressed = visionButton.get();

    neoDrive.drive(rightSpeed, leftSpeed, 1.0, true);

    vision.updateVisionVals();
    if (visionButtonPressed) {
      vision.runVision();
    }
    shooter.setToShoot(manipulatorStickSpeed);
    

      // }


    // while (vision.foundTarget() && vision.linedUp()) {
    //     vision.writeDistanceAndAngle();
    //     // In the future, this method will double check that the robot is in shooting range
    // }
  }

  @Override
  public void testPeriodic() {
  }
}
