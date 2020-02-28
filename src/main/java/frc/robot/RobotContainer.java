package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import frc.robot.subsystems.*;

public class RobotContainer {

    //mechanical subsystems
    public Drivetrain drive = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public Indexer indexer = new Indexer();
    public ControlPanel controlPanel = new ControlPanel();
    public Webcams webcams = new Webcams();

    //other portions
    public AHRS ahrs;
    

    private static RobotContainer robotContainer;

    protected RobotContainer() {
        ahrs = new AHRS(SerialPort.Port.kUSB);
		ahrs.reset();
    }

    public static RobotContainer getInstance() {
        if (robotContainer == null) {
            robotContainer = new RobotContainer();
        }
        return robotContainer;
    }

    public void runTeleopControls() {
        drive.teleopControl();
        climber.teleopControl();
        controlPanel.teleopControl();
        webcams.teleopControl();
        runPowerCellSystem();
    }

    public void runPowerCellSystem() {
        intake.teleopControl();
        shooter.teleopControl();
        indexer.teleopControl();
    }

    public void robotUpdateSystems() {
        drive.periodic();
        intake.periodic();
        shooter.periodic();
        climber.periodic();
        indexer.periodic();
        controlPanel.periodic();
        webcams.periodic()
        
    }

    public void runSelectSystem() {
        drive.teleopControl();
        drive.periodic();
    }
}