package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter.shooterStates;

public class Autonomous {

    private RobotContainer robotContainer = RobotContainer.getInstance();

    int autonomousStep = 0;
    int DesieredStep = 1;
    Timer autonomousTimer;
    double transitionTime = 0;
    PID drive;
    double leftInchesRecord;    
    double rightInchesRecord;    

    private final String DriveAndShoot = "DriveAndShoot";
    private final String StraightShot = "StraightShot";
    private final String SimpleDrive = "SimpleDrive";
    private final String AutoNav = "AutoNav";

    private final SendableChooser<String> autonomousChooser = new SendableChooser<>();

    enum AutonomousModes {
        FAR_RIGHT_SHOT, CENTERED_SHOT, FAR_RIGHT_SHOT_COLLECT_SHOT, CENTERED_SHOT_COLLECT_SHOT
    }

    public Autonomous() {
        autonomousTimer = new Timer();
        autonomousTimer.start();
        drive = new PID(0.03, 0, 0);
        autonomousStep = 0;
        autonomousChooser.setDefaultOption(DriveAndShoot, DriveAndShoot);
        autonomousChooser.addOption(StraightShot, StraightShot);
        autonomousChooser.addOption(SimpleDrive, SimpleDrive);
        SmartDashboard.putData(autonomousChooser);
    }

    private void moveForward() {
        // drive forward a specified amount
        drive.setError(-robotContainer.ahrs.getAngle());
        drive.update();
        robotContainer.drive.setDriveSpeeds(0.5, 0.5);
    }

    private void moveStop() {
        // stop driving
        drive.setError(-robotContainer.ahrs.getAngle());
        drive.update();
        robotContainer.drive.setDriveSpeeds(0, 0);
    }

    private void moveTurnLeft() {
        // left turn. only left motor is powered right now
        drive.setError(-robotContainer.ahrs.getAngle());
        drive.update();
        robotContainer.drive.setDriveSpeeds(0, 0.2);
    }
    private float testDirecton () {
        return robotContainer.ahrs.getYaw();
    }

    private void moveTurnRight() {
        // right turn. only left motor is powered right now
        drive.setError(-robotContainer.ahrs.getAngle());
        drive.update();
        robotContainer.drive.setDriveSpeeds(0.2, -0.2);
    }

    private void intakeOut() {
        // right turn. only left motor is powered right now
        drive.update();
        robotContainer.intake.setState(Intake.IntakeStates.OUT);
    }

    private void intakeIdle() {
        // right turn. only left motor is powered right now
        drive.update();
        robotContainer.intake.setState(Intake.IntakeStates.IDLE);
}

    public void runAutonomous() {
        robotContainer.ahrs.zeroYaw();
        AutoNavBounce();
    }
    //simple auto that will drive forward for x time and then shoot against wall
    public void driveAndShoot() {
        SmartDashboard.putNumber("time", autonomousTimer.get() - transitionTime);
        switch (autonomousStep) {
            case 0:
            /*should be driving forward 10 feet, still needs to be tuned */
                drive.setError(-robotContainer.ahrs.getAngle());
                drive.update();
                robotContainer.drive.setDriveSpeeds(0.2 + drive.getOutput(), 0.2 - drive.getOutput());
                if (robotContainer.drive.getLeftInches() > 96/*autonomousTimer.get() - transitionTime > 1000*/) {
                    switchStep();
                }
                break;
            case 1:
            //stops driving -- no brake
                robotContainer.drive.setDriveSpeeds(0, 0);
                if (autonomousTimer.get() - transitionTime > 0.5) {
                    switchStep();
                }
                break;
            case 2:
            //begins to spin up the shooters
                robotContainer.shooter.setLowerShooterSpeed(1);
                robotContainer.shooter.setShooterSpeed(0.61);
                robotContainer.shooter.setLowerShooterState(shooterStates.BASE_SPEED);
                robotContainer.shooter.setState(shooterStates.BASE_SPEED);
                if (autonomousTimer.get() - transitionTime > 1) {
                    switchStep();
                }
                break;
            case 3:
            //begins indexing/shooting
                robotContainer.indexer.setState(Indexer.IndexerState.BACKWARD);
                if (autonomousTimer.get() - transitionTime > 10) {
                    switchStep();
                }
                break;
            case 4:
            //stops all mechanisms
                robotContainer.indexer.setState(Indexer.IndexerState.IDLE);
                robotContainer.shooter.setLowerShooterState(shooterStates.OFF);
                robotContainer.shooter.setState(shooterStates.OFF);
                break;

                
        }
    }

    public void simpleDrive() {
        switch (autonomousStep) {
            case 0:
            /*should be driving forward 10 feet, still needs to be tuned */
                drive.setError(-robotContainer.ahrs.getAngle());
                drive.update();
                robotContainer.drive.setDriveSpeeds(0.2 + drive.getOutput(), 0.2 - drive.getOutput());
                if (robotContainer.drive.getLeftInches() > 96/*autonomousTimer.get() - transitionTime > 1000*/) {
                    switchStep();
                }
                break;
            case 1:
            //stop driving --- coasts right now
                robotContainer.drive.setDriveSpeeds(0, 0);
                break;
        }
    }


    double howFarRight() {
        return robotContainer.drive.getRightInches()- rightInchesRecord;
    }

    double howFarLeft() {
        return robotContainer.drive.getLeftInches() - rightInchesRecord;
    }
    public void AutoNavSlalomPath() {
        switch (autonomousStep) {
            case 0: {
                //Drive Forward
                moveForward();
                if (howFarLeft() > 40) {
                    switchStep();
                }
                break;
            }
            case 1: {
                //Stop Driving
                moveStop();
                switchStep();
                break; 
            }   

            case 2: {
                //Turn Left
                moveTurnLeft();
                if (testDirecton() < -45) {
                    switchStep();
                }
                break;
            }
             case 3: {
                //Stop Driving
                moveStop();
                switchStep();
                break;
             } 
             case 4: {
                //Drive Forward
                moveForward();
                if (howFarLeft() > 30) {
                    switchStep();
                }
                break;
            }
            case 5: {
                //Stop Driving
                moveStop();
                switchStep();
                break; 
            }
            case 6: {
                //Left Turn
                moveTurnRight();
                if (howFarLeft() > 45) {
                    switchStep();
                }
                break;
            }
            case 7: {
                //Stop Driving
                moveStop();
                switchStep();
                break; 
            }
            case 8: {
                //Drive Forward
                moveForward();
                if (howFarLeft() > 140) {
                    switchStep();
                }
                break;
            }
            case 9: {
                //Stop Driving
                moveStop();
                break; 
            }
        }
    }
    public void AutoNavBarrelRacing() {
            switch (autonomousStep) {
            case 0: {
                //Drive Forward
                moveForward();
                if (howFarLeft() > 40) {
                    switchStep();
                }
                break;
            }
            case 1: {
                //Stop Driving
                moveStop();
                break; 
                }
        }
        
    }
    public void AutoNavBounce() {
        
        switch (autonomousStep) {
        case 0: {
            //Drive Forward (5 ft)
            moveForward();
            if (howFarLeft() > 50) {
                switchStep();
            }
            break;
        }
        case 1: {
            //Stop Driving
            moveStop();
            switchStep();
            break; 
            }
        case 2: {
            //Drive Left (~90°)
            moveTurnLeft();
            if (howFarRight() > 40) {
                switchStep();
            }
            break;
        }
        case 3: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 4: {
            //Drive Forward (3 ft)
            moveForward();
            intakeOut();
            if (howFarLeft() > 1 && howFarLeft() < 3) {
                switchStep();
            }
            break;   
        }
        case 5: {
            //Stop Driving (Location = A3)
            moveStop();
            intakeIdle();
            switchStep();
            break;
        }    
        case 6: {
            //Turn Right (~150°)
            moveTurnRight();
            if (howFarLeft() > 0.1) {
                switchStep();
            
                
            }
            break;
        }
        case 7: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 8: {
            //Drive Straight (20 ft)
            moveForward();
            if (howFarLeft() > 130) {
                switchStep();
            }
            break;
        }
        case 9: {
            //Stop Driving (Location: ~E4)
            moveStop();
            switchStep();
            break;
        }
        case 10: {
            //Turn Left (~90°)
            moveTurnLeft();
            if (howFarRight() > 50) {
                switchStep();
            }
            break;
        }
        case 11: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 12: {
            //Drive Forward (5ft)
            moveForward();
            if (howFarLeft() > 30) {
                switchStep();
            }
            break;
        }
        case 13: {
            //Stop Driving (Location = ~E6)
            moveStop();
            switchStep();
            break;
        }
        case 14: {
            //Turn Left (~90°)
            moveTurnLeft();
            if (howFarRight() > 40) {
                switchStep();
            }
            break;
        }
        case 15: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 16: {
            //Drive Forward (20 ft)
            moveForward();
            if (howFarLeft() > 100) {
                switchStep();
            }
            break;
        }
        case 17: {
            
            //Stop Driving (Location = A6)
            moveStop();
            switchStep();
            break;
        }
        /*case 18: {
            //Turn Right (~180°)
            moveTurnRight();
            if (howFarLeft() > 80) {
                switchStep();
            }
            break;
        }
        case 19: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 20: {
            //Drive Forward (20 ft)
            moveForward();
            if (howFarLeft() > 120) {
                switchStep();
            }
            break;
        }
        case 21: {
            //Stop Driving (Location = ~E6)
            moveStop();
            switchStep();
            break;
        }
        case 22: {
            //Turn Left (~90°)
            moveTurnLeft();
            if (howFarRight() > 40) {
                switchStep();
            }
            break;
        }
        case 23: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 24: {
            //Drive Forward (10 ft)
            moveForward();
            if (howFarLeft() > 60) {
                switchStep();
            }
            break;
        }
        case 25: {
            //Stop Driving (Location = E9)
            moveStop();
            switchStep();
            break;
        }
        case 26: {
            //Turn Left (~90°)
            moveTurnLeft();
            if (howFarRight() > 40) {
                switchStep();
            }
            break;
        }
        case 27: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 28: {
            //Drive Forward (20 ft)
            moveForward();
            if (howFarLeft() > 120) {
                switchStep();
            }
            break;
        }
        case 29: {
            //Stop Driving (Location = A9)
            moveStop();
            switchStep();
            break;
        }
        case 30: {
            //Turn Right (~180°)
            moveTurnRight();
            if (howFarLeft() > 80) {
                switchStep();
            }
            break;
        }
        case 31: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 32: {
            //Drive Forward (10 ft)
            moveForward();
            if (howFarLeft() > 60) {
                switchStep();
            }
            break;
        }
        case 33: {
            //Stop Driving (Location = ~C9)
            moveStop();
            switchStep();
            break;
        }
        case 34: {
            //Turn Left (~90°)
            moveTurnLeft();
            if (howFarRight() > 40) {
                switchStep();
            }
            break;
        }
        case 35: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 36: {
            //Drive Forward (5 ft)
            moveForward();
            if (howFarLeft() > 30) {
                switchStep();
            }
            break;
        }
        case 37: {
            //Stop Driving (Location = Finish Zone)
            moveStop();
            break;
        }*/
    
        }
    }
    
    private void switchStep() {
        rightInchesRecord = robotContainer.drive.getRightInches(); 
        leftInchesRecord = robotContainer.drive.getLeftInches(); 
        autonomousStep ++;
        transitionTime = autonomousTimer.get();
        SmartDashboard.putNumber("CurrentCase", autonomousStep);
        SmartDashboard.putNumber("HowFarLeft", howFarLeft());
        SmartDashboard.putNumber("HowFarRight", howFarRight());
        SmartDashboard.putNumber("CurrentCase", robotContainer.drive.getRightInches());
        SmartDashboard.putNumber("getLeftInches", robotContainer.drive.getLeftInches());
    }
    private void switchStepByCase(int makeItMakeSense) {
        autonomousStep = makeItMakeSense;
        DesieredStep ++;
        transitionTime = autonomousTimer.get();
        SmartDashboard.putNumber("CurrentCase", autonomousStep);
        SmartDashboard.putNumber("DesiredStep", DesieredStep);
    }
    // public void getAutonomousCommand() {
    //     TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    //     config.setKinematics(robotContainer.drive.getKinematics());

    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
    //         config
    //     );
    // }

    // RamseteCommand command = new RamseteCommand {
        
    // }
}
