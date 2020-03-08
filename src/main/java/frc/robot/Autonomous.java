package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.shooterStates;


public class Autonomous {

    private RobotContainer robotContainer = RobotContainer.getInstance();

    int autonomousStep = 0;
    Timer autonomousTimer;
    double transitionTime = 0;
    PID drive;

    private final SendableChooser<AutonomousModes> autonomousChooser = new SendableChooser<>();

    public enum AutonomousModes {
        SHOOT_DRIVE, SIMPLE_DRIVE, DIRECT_SHOT_TO_TRENCH
    }

    //import path jsons
    String angledToTrenchJSON = "paths/output/AngledToTrench.wpilib.json";
    String straightToTrenchJSON = "paths/output/DirectShotToTrench";



    double startPoint = 0;

    public Autonomous() {
        //initialize Auton settings
        autonomousTimer = new Timer();
        autonomousTimer.start();
        autonomousStep = 0;

        //drive and turn PID inits
        drive = new PID(0.03, 0, 0);

        //update Shuffleboard
        autonomousChooser.setDefaultOption("DriveAndShoot", AutonomousModes.SHOOT_DRIVE);
        autonomousChooser.addOption("DriveForward", AutonomousModes.SIMPLE_DRIVE);
        autonomousChooser.addOption("InitialPathTest", AutonomousModes.DIRECT_SHOT_TO_TRENCH);
        SmartDashboard.putData(autonomousChooser);

        //handle trajectory configuration
        try {
            Path angledToTrenchPath = Filesystem.getDeployDirectory().toPath().resolve(angledToTrenchJSON);
            Trajectory angledToTrench = TrajectoryUtil.fromPathweaverJson(angledToTrenchPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + angledToTrenchJSON, ex.getStackTrace());
        }
        try {
            Path straightToTrenchPath = Filesystem.getDeployDirectory().toPath().resolve(straightToTrenchJSON);
            Trajectory straightToTrench = TrajectoryUtil.fromPathweaverJson(straightToTrenchPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + straightToTrenchJSON, ex.getStackTrace());
        }
    }

    public void runAutonomous() {
       // driveAndShoot();
        
        if (autonomousChooser.getSelected() == AutonomousModes.SHOOT_DRIVE) {
            driveAndShoot();
        } else if (autonomousChooser.getSelected() == AutonomousModes.SIMPLE_DRIVE) {
            simpleDrive();
        } else if (autonomousChooser.getSelected() == AutonomousModes.DIRECT_SHOT_TO_TRENCH) {
            directShotToTrenchReturn();
        }
        
    }

    public void directShotToTrenchReturn() {
        switch (autonomousStep) {
            case 0:
                drive.reset();
                switchStep();
            case 1:
                //ActivateVision
                if (true/*if vision is aligned*/) {
                    switchStep();
                }
                break;
            case 2: 
                //Shoot
                if (true) {
                    switchStep();
                }
                break;
            case 3:
                //drive to trench
                if (robotContainer.drive.getLeftInches() > 96) {
                    robotContainer.intake.setState(IntakeStates.IN);
                }
                if (true/*trajectory is complete*/) {
                    switchStep();
                }
                break;
            case 4:
                //backup
                robotContainer.intake.setState(IntakeStates.IDLE);
                drive.setError(-robotContainer.ahrs.getAngle());
                drive.update();
                robotContainer.drive.setDriveSpeeds(-0.2 + drive.getOutput(), -0.2 - drive.getOutput());
                if (robotContainer.drive.getLeftInches() < -60/*autonomousTimer.get() - transitionTime > 1000*/) {
                    switchStep();
                }
                break;
            case 5:
                //drive to base trajectory
                if (true/*complete*/) {
                    switchStep();
                }
                break;
            case 6:
                robotContainer.shooter.setLowerShooterSpeed(1);
                robotContainer.shooter.setShooterSpeed(0.61);
                robotContainer.shooter.setLowerShooterState(shooterStates.BASE_SPEED);
                robotContainer.shooter.setState(shooterStates.BASE_SPEED);
                if (autonomousTimer.get() - transitionTime > 1) {
                    switchStep();
                }
                break;
            case 7:
                robotContainer.indexer.setState(Indexer.IndexerState.BACKWARD);
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

    public void stupidFast() {
        switch (autonomousStep) {
            case 0:
                startPoint = robotContainer.drive.getLeftInches();
                switchStep();
            case 1:
                drive.setError(-robotContainer.ahrs.getAngle());
                drive.update();
                robotContainer.drive.setDriveSpeeds(1 + drive.getOutput(), 1 - drive.getOutput());
                if (robotContainer.drive.getLeftInches() - startPoint > 96/*autonomousTimer.get() - transitionTime > 1000*/) {
                    switchStep();
                }
                break;
            case 2:
                robotContainer.drive.setDriveSpeeds(0, 0);
        }
    }

    public void simpleDrive() {
        switch (autonomousStep) {
            case 0:
                drive.setError(-robotContainer.ahrs.getAngle());
                drive.update();
                robotContainer.drive.setDriveSpeeds(0.2 + drive.getOutput(), 0.2 - drive.getOutput());
                if (robotContainer.drive.getLeftInches() > 96) {
                    switchStep();
                }
                break;
            case 1:
            //stop driving --- coasts right now
                robotContainer.drive.setDriveSpeeds(0, 0);
                break;
        }
    }

    private void switchStep() {
        autonomousStep ++;
        transitionTime = autonomousTimer.get();
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