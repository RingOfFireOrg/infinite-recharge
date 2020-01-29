package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

public class Encoders {
    private Encoder leftEncoder = new Encoder(RobotMap.DRIVE_TRAIN_LEFT_ENCODER_A,
            RobotMap.DRIVE_TRAIN_LEFT_ENCODER_B, false, Encoder.EncodingType.k1X);
    private Encoder rightEncoder = new Encoder(RobotMap.DRIVE_TRAIN_RIGHT_ENCODER_A,
            RobotMap.DRIVE_TRAIN_RIGHT_ENCODER_B, false, Encoder.EncodingType.k1X);


    public double getEncoderVal(Encoder encoder) {
        return encoder.getDistance();
    }
}