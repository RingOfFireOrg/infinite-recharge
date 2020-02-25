  package frc.robot;

import com.revrobotics.CANEncoder;

public class Encoders {
    CANEncoder driveTrainRightEncoderA;
    CANEncoder driveTrainRightEncoderB;
    CANEncoder driveTrainLeftEncoderA;
    CANEncoder driveTrainLeftEncoderB;

    public void initEncoders(CANEncoder rightEncoderA, CANEncoder rightEncoderB,
                             CANEncoder leftEncoderA, CANEncoder leftEncoderB) {
        this.driveTrainRightEncoderA = rightEncoderA;
        this.driveTrainRightEncoderB = rightEncoderB;
        this.driveTrainLeftEncoderA = leftEncoderA;
        this.driveTrainLeftEncoderB = leftEncoderB;
    }

    public double getSingleEncoderVal(CANEncoder encoder) {
        return encoder.getPosition();
    }
}