package frc.robot;

import frc.robot.subsystems.*;

public class RobotContainer {

    public Drivetrain drive = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public Indexer indexer = new Indexer();
    public ControlPanel controlPanel = new ControlPanel();
    

    private static RobotContainer robotContainer;

    protected RobotContainer() {

    }

    public static RobotContainer getInstance() {
        if (robotContainer == null) {
            robotContainer = new RobotContainer();
        }
        return robotContainer;
    }

    public void runTeleopControls() {
        drive.teleopControl();
        intake.teleopControl();
        shooter.teleopControl();
        climber.teleopControl();
        indexer.teleopControl();
        controlPanel.teleopControl();
    }

    public void robotUpdateSystems() {
        drive.periodic();
        intake.periodic();
        shooter.periodic();
        climber.periodic();
        indexer.periodic();
        controlPanel.periodic();
    }

    public Drivetrain getDrivetrain() {
        return drive;
    }
}