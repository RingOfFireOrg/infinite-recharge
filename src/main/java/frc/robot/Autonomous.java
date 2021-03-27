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

    public void runAutonomous() {
        AutoNav();
        /*
        if (autonomousChooser.getSelected() == DriveAndShoot) {
            driveAndShoot();
        } else if (autonomousChooser.getSelected() == StraightShot) {
            simpleShoot();
        } else if (autonomousChooser.getSelected() == SimpleDrive) {
            simpleDrive();
        }
        */
    }

    //simple auto that will shoot immediately
    public void simpleShoot() {
        switch (autonomousStep) {
            case 0:
                robotContainer.shooter.setLowerShooterSpeed(1);
                robotContainer.shooter.setShooterSpeed(1);
                robotContainer.shooter.setLowerShooterState(shooterStates.BASE_SPEED);
                robotContainer.shooter.setState(shooterStates.BASE_SPEED);
                if (autonomousTimer.get() - transitionTime > 1000) {
                    switchStep();
                }
                break;
            case 1:
                robotContainer.indexer.setState(Indexer.IndexerState.FORWARD);
                if (autonomousTimer.get() - transitionTime > 4000) {
                    switchStep();
                }
                break;
            case 2:
                robotContainer.indexer.setState(Indexer.IndexerState.IDLE);
                robotContainer.shooter.setLowerShooterState(shooterStates.OFF);
                robotContainer.shooter.setState(shooterStates.OFF);
                break;
        }
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

    private void moveForward() {
            /*should be driving forward 10 feet, still needs to be tuned */
                drive.setError(-robotContainer.ahrs.getAngle());
                drive.update();
                robotContainer.drive.setDriveSpeeds(0.5, 0.5);
    }
    private void moveStop() {
           //stop driving --- coasts right now
            drive.setError(-robotContainer.ahrs.getAngle());
            drive.update();
            robotContainer.drive.setDriveSpeeds(0, 0);
    }
    private void moveTurnLeft() {
        drive.setError(-robotContainer.ahrs.getAngle());
        drive.update();
        robotContainer.drive.setDriveSpeeds(0, 0.2);
    }

    double howFarRight() {
        return robotContainer.drive.getRightInches()- rightInchesRecord;
    }

    double howFarLeft() {
        return robotContainer.drive.getLeftInches() - rightInchesRecord;
    }
    public void AutoNav() {
        switch (autonomousStep) {
            case 0: {
            /*should be driving forward 10 feet, still needs to be tuned */
                moveForward();
                if (howFarLeft() > 50/*autonomousTimer.get() - transitionTime > 1000*/) {
                    switchStep();
                }
                break;
            }
            case 1: {
                //stop driving --- coasts right now
                moveStop();
                switchStep();
                break; 
            }   

            case 2: {
                /*Turn left */
                moveTurnLeft();
                if (howFarRight() > 25/*autonomousTimer.get() - transitionTime > 1000*/) {
                    switchStep();
                }
                break;
            }
             case 3: {
            //stop driving --- coasts right now
                moveStop();
                break;
             } 
        }
}
    
    private void switchStep() {
        rightInchesRecord = robotContainer.drive.getLeftInches(); 
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
