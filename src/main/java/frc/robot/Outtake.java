package frc.robot;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Outtake {

    Servo left = new Servo(RobotMap.SERVO_LEFT);
    Servo right = new Servo(RobotMap.SERVO_RIGHT);
    private static final int OPEN = 170;
    private static final int CLOSED = 5;

        public Outtake () 
        {
            left.setAngle(Outtake.CLOSED);
            right.setAngle(opposite(Outtake.CLOSED));

        }
        public void open()
        {
            left.setAngle(Outtake.OPEN);
            right.setAngle(opposite(Outtake.OPEN));
        }
        public void close()
        {
            left.setAngle(Outtake.CLOSED);
            right.setAngle(opposite(Outtake.CLOSED));
        }
        private int opposite(int value)
        {
           int temp = 180 - value;
            return temp;
        }
}