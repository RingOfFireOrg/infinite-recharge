package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchGroup {

    enum switchState {
        LOWER, MIDDLE, UPPER
    }

    private switchState state;

    private int lowerBoundPort, upperBoundPort;

    private DigitalInput lowerSwitch, upperSwitch;

    public LimitSwitchGroup (int lowerBoundport, int upperBoundport) {
        state = switchState.MIDDLE;
        this.lowerBoundPort = lowerBoundport;
        this.upperBoundPort = upperBoundport;
        lowerSwitch = new DigitalInput(lowerBoundport);
        upperSwitch = new DigitalInput(upperBoundPort);
    }

    public switchState getState() {
        if (lowerSwitch.get() == true) {
            state = switchState.LOWER;
        } else if (upperSwitch.get() == true) {
            state = switchState.UPPER;
        } else {
            state = switchState.MIDDLE;
        }
        return state;
    }
}