package com.team1806.frc2020.subsystems;

import com.team1806.frc2020.Constants;
import edu.wpi.first.wpilibj.Relay;

public class Flashlight extends Subsystem {

    public static Flashlight FLASHLIGHT = new Flashlight();
    private PeriodicIO mPeriodicIO;
    private FlashlightState mFlashlightState;
    private Relay mRelay;
    private Flashlight() {
        mRelay = new Relay(Constants.kFlashlightRelayChannel, Relay.Direction.kForward);
        mPeriodicIO = new PeriodicIO();
        setState(FlashlightState.kOff);
    }

    public static Flashlight GetInstance() {
        return FLASHLIGHT;
    }

    private void setState(FlashlightState state) {
        mPeriodicIO.flashlightState = state;
        mFlashlightState = state;
    }

    public void setFlashlightOn(boolean On) {
        if (On) {
            setState(FlashlightState.kOn);
        } else {
            setState(FlashlightState.kOff);
        }
    }

    public void readPeriodicInputs() {

    }

    public void writePeriodicOutputs() {
        switch (mPeriodicIO.flashlightState) {
            default:
            case kOff:
                mRelay.set(Relay.Value.kOff);
            case kOn:
                mRelay.set(Relay.Value.kOn);
        }
    }

    public void stop() {
        setState(FlashlightState.kOff);
    }

    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
    }


    enum FlashlightState {
        kOff, kOn
    }

    private class PeriodicIO {
        public FlashlightState flashlightState;
    }

}
