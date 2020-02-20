package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    public Drivetrain drive = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public Indexer indexer = new Indexer();
    

    private static RobotContainer robotContainer;

    protected RobotContainer() {

    }

    public static RobotContainer getInstance() {
        if (robotContainer == null) {
            robotContainer = new RobotContainer();
        }
        return robotContainer;
    }

    public void robotUpdateSystems() {
        drive.periodic();
    }

    public Drivetrain getDrivetrain() {
        return drive;
    }
}