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
    double FEET = 8.50; // To go one FEET, the robot encoder has to read ~8.50 inches of the wheel
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

    private void moveBackward() {
        // drive backward a specified amount
        drive.setError(-robotContainer.ahrs.getAngle());
        drive.update();
        robotContainer.drive.setDriveSpeeds(-0.5,-0.5);
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
    public float getabsoluteDirection() {
        return robotContainer.ahrs.getYaw();
    }

    private void moveTurnRight() {
        // right turn. only left motor is powered right now
        drive.setError(-robotContainer.ahrs.getAngle());
        drive.update();
        robotContainer.drive.setDriveSpeeds(0.1, -0.1);
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
        AutoNavBounce();
    }
    //simple auto that will drive forward for x time and then shoot against wall
    public void driveAndShoot() {
        SmartDashboard.putNumber("time", autonomousTimer.get() - transitionTime);
        switch (autonomousStep) {
            case 0:
            /*should be driving forward 10 FEET, still needs to be tuned */
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
            /*should be driving forward 10 FEET, still needs to be tuned */
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
        return robotContainer.drive.getLeftInches() - leftInchesRecord;
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
                if (getabsoluteDirection() < -45) {
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
    
    

        public void AutoNavBarrelRace() {
            switch (autonomousStep) {
                case 0: {
                    //Drive Forward 13 ft
                    moveForward();
                    if (howFarLeft() > 13.5*FEET) {
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
                    //Turn Right 90°
                    moveTurnRight();
                    if (getabsoluteDirection() > 90) {
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
                //Drive Forward 3 FEET
                moveForward();
                if (howFarLeft() > 3*FEET) {
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
                    //Turn Right 90° 
                    moveTurnRight();
                    if (getabsoluteDirection() > 179) { //needs to be >180, bc the gyro max reading is 180
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
                //Drive Forward 3 FEET
                moveForward();
                if (howFarLeft() > 3*FEET) {
                    switchStep();
                    }
                    break;
                }
                case 9: {
                //Stop Driving
                moveStop();
                switchStep();
                break;
                }
                case 10: {
                    //Turn Right 90°
                    moveTurnRight();
                    if (getabsoluteDirection() > -90) {
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
                    //Drive Forward 3 ft
                    moveForward();
                    if (howFarLeft() > 3*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 13: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 14: {
                    //Turn Right 75°
                    moveTurnRight();
                    if (getabsoluteDirection() > -15) {
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
                    //Drive Forward 13 FEET
                    moveForward();
                    if (howFarLeft() > 13*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 17: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 18: {
                    // Turn left 75°
                    moveTurnLeft();
                    if (getabsoluteDirection() < -90) {
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
                    //Drive Forward 2.5 FEET
                    moveForward();
                    if (howFarLeft() > 2.5*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 21: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 22: {
                    //Turn Left 90
                    moveTurnLeft();
                    if (getabsoluteDirection() < -179) { //gyro reads from -180 to +180
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
                    //Drive Forward 3 FEET
                    moveForward();
                    if (howFarLeft() > 3*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 25: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 26: {
                    //Turn Left 135
                    moveTurnLeft();
                    if (getabsoluteDirection() > 150) { 
                        //gyro either at 180 or -180, so >45 as the absolute direction would stop anywhere between 180 and 45. 
                        moveTurnLeft();
                    }
                    else if(getabsoluteDirection() < 45) {
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
                    //Drive Forward 9 FEET
                    moveForward();
                    if (howFarLeft() > 9*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 29: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 30: {
                    //Turn Left 45
                    moveTurnLeft();
                    if (getabsoluteDirection() < 0) {
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
                    //Drive Forward 5 FEET
                    moveForward();
                    if (howFarLeft() > 5*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 33: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 34: {
                    //Turn Left 90
                    moveTurnLeft();
                    if (getabsoluteDirection() < -90) {
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
                    //Drive Forward 5 FEET
                    moveForward();
                    if (howFarLeft() > 5*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 37: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 38: {
                    //Turn Left 90 
                    moveTurnLeft(); 
                    if (getabsoluteDirection() < -179) {
                        switchStep();
                    }
                    break;
                }
                case 39: {
                    //Stop Driving
                    moveStop();
                    switchStep();
                    break;
                }
                case 40: {
                    //Drive Forward 27 FEET
                    moveForward();
                    if (howFarLeft() > 27*FEET) {
                        switchStep();
                    }
                    break;
                }
                case 41: {
                    //Stop Driving
                    moveStop();
                    break;
                }
            }
        }

    public void AutoNavBounce() {
        
        switch (autonomousStep) {
        case 0: {
            //Drive Forward (6 ft)
            moveForward();
            if (howFarLeft() > 6*FEET) {
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
            if (getabsoluteDirection() < -88) {
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
            if (howFarLeft() > 3*FEET) {
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
            moveTurnRight();
            if (getabsoluteDirection() > 50) {
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
            if (howFarLeft() > 12*FEET) {
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
            moveTurnLeft();
            if (getabsoluteDirection() < 10) {
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
            if (howFarLeft() > 2.5*FEET) {
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
            moveTurnLeft();
            if (getabsoluteDirection() < -75) {
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
            //Drive Forward (10 ft)
            moveForward();
            intakeOut();
            if (howFarLeft() > 80) {
                switchStep();
            }
            break;
        }
        case 17: {

            //Stop Driving (Location = A6)
            moveStop();
            intakeIdle();
            switchStep();
            break;
        }
        case 18: {
            //Drive Backward (10 ft)
            moveBackward();
            if (howFarLeft() < -10*FEET) {
                switchStep();
            }
            break;
        }
        case 19: {
            //Stop Driving (Location = ~E6)
            moveStop();
            switchStep();
            break;
        }
        case 20: {
            //Turn Right (~90°)
            moveTurnRight();
            if (getabsoluteDirection() > 0) {
                switchStep();
            }
            break;
        }
        case 21: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 22: {
            //Drive Forward (7 ft)
            moveForward();
            if (howFarLeft() > 7*FEET) {
                switchStep();
            }
            break;
        }
        case 23: {
            //Stop Driving (Location = E9)
            moveStop();
            switchStep();
            break;
        }
        case 24: {
            //Turn Left (~90°)
            moveTurnLeft();
            if (getabsoluteDirection() < -85) {
                switchStep();
            }
            break;
        }
        case 25: {
            //Stop Driving
            moveStop();
            switchStep();
            break;
        }
        case 26: {
            //Drive Forward (10 ft)
            moveForward();
            intakeOut();
            if (howFarLeft() > 10*FEET) {
                switchStep();
            }
            break;
        }
        case 27: {
            //Stop Driving (Location = A9)
            moveStop();
            intakeIdle();
            switchStep();
            break;
        }
        case 28: {
            //Drive Backward (5 ft)
            moveBackward();
            if (howFarLeft() < -5*FEET) {
                switchStep();
            }
            break;
        }
        case 29: {
            //Stop Driving (Location = ~C9)
            moveStop();
            switchStep();
            break;
        }
        case 30: {
            //Turn Right (~90°)
            moveTurnRight();
            if (getabsoluteDirection() > 0) {
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
            //Drive Forward (4 ft)
            moveForward();
            if (howFarLeft() > 5*FEET) {
                switchStep();
            }
            break;
        }
        case 33: {
            //Stop Driving (Location = Finish Zone)
            moveStop();
            break;
        }
    
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
        SmartDashboard.putNumber("getRightInches", robotContainer.drive.getRightInches());
        SmartDashboard.putNumber("getLeftInches", robotContainer.drive.getLeftInches()); 
        SmartDashboard.putNumber("NavXAngle", getabsoluteDirection());
    }
    private void switchStepByCase(int makeItMakeSense) {
        autonomousStep = makeItMakeSense;
        DesieredStep ++;
        transitionTime = autonomousTimer.get();
        SmartDashboard.putNumber("CurrentCase", autonomousStep);
        SmartDashboard.putNumber("DesiredStep", DesieredStep);
    }
    // public void getAutonomousCommand() {
    //     TrajectoryConfig config = new TrajectoryConfig(Units.FEETToMeters(2), Units.FEETToMeters(2));
 //config.setKinematics(robotContainer.drive.getKinematics());

    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
    //         config
    //     );
    // }

    // RamseteCommand command = new RamseteCommand {
        
    // }
}
