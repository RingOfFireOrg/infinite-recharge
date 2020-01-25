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
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Robot extends TimedRobot {

  Joystick rightstick = new Joystick(0);
  Joystick leftstick = new Joystick(1);
  Joystick manipulatorStick = new Joystick(2);
  JoystickButton visionButton = new JoystickButton(rightstick, 1);
  
  AHRS ahrs;
  NeoTankDrive neoDrive;
  Vision vision;
  PID visionLineupPid;

  double currentGyroAngle;
  double drivetrainRotationMagnitude;
  
  public CANSparkMax frontLeftMotor;
  public CANSparkMax frontRightMotor;
  public CANSparkMax backRightMotor;
  public CANSparkMax backLeftMotor;
  public CANSparkMax neoPrototypeMotor;

  boolean lookingForVisionTarget = false;

  @Override
  public void robotInit() {
    ahrs = new AHRS(SerialPort.Port.kUSB);
    ahrs.reset();

    neoDrive = new NeoTankDrive();

    visionLineupPid = new PID(0.003, 0.0005 ,0);
    visionLineupPid.setOutputRange(-0.2, 0.2);
    
    vision = new Vision();
    frontLeftMotor = new CANSparkMax(RobotMap.NEO_FRONT_LEFT, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(RobotMap.NEO_FRONT_RIGHT, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(RobotMap.NEO_BACK_RIGHT, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(RobotMap.NEO_BACK_LEFT, MotorType.kBrushless);
    neoPrototypeMotor = new CANSparkMax(RobotMap.NEO_PROTOTYPE, MotorType.kBrushless);

    lookingForVisionTarget = false;
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
    double currentTargetDist = vision.getVisionTargetDistance();
    boolean visionButtonPressed = visionButton.get();
    currentGyroAngle = ahrs.getAngle();

    neoPrototypeMotor.set(manipulatorStickSpeed);
    neoDrive.drive(rightSpeed, leftSpeed, 1.0, true);

    vision.updateVisionVals(); 
    
    if (visionButtonPressed){
      // while (vision.foundTarget() && !vision.linedUp()) {
          if (!lookingForVisionTarget) {
              lookingForVisionTarget = true;
              visionLineupPid.reset();
          }
          visionLineupPid.setError(vision.getVisionTargetAngle(currentGyroAngle));
          visionLineupPid.update();
          drivetrainRotationMagnitude = visionLineupPid.getOutput();

            if (Math.abs(vision.tx) > 2) {
              neoDrive.setSpeed(-drivetrainRotationMagnitude, -drivetrainRotationMagnitude);
              SmartDashboard.putBoolean("Vision status", lookingForVisionTarget);
              SmartDashboard.putString("Target status", "Going to target!");
          } else {
              // neoDrive.stop();
              lookingForVisionTarget = false;
              SmartDashboard.putString("Target status", "Found target!");
          }
      // }
    }

    // while (vision.foundTarget() && vision.linedUp()) {
    //     vision.writeDistanceAndAngle();
    //     // In the future, this method will double check that the robot is in shooting range
    // }
  }

  @Override
  public void testPeriodic() {
  }
}
