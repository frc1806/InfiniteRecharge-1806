package com.team1806.frc2020;

import com.team1806.frc2020.loops.ILooper;
import com.team1806.frc2020.loops.Loop;
import com.team1806.frc2020.loops.Looper;
import com.team1806.frc2020.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
    public static SubsystemManager mInstance = null;

    private List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();
    private boolean pauseLoops = false;

    private SubsystemManager() {
    }

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }

        return mInstance;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem s : mAllSubsystems) {
            ret_val &= s.checkSystem();
        }

        return ret_val;
    }

    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }

    public void setSubsystems(Subsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    public void zeroSensors() {
        pauseLoops = true;
        mAllSubsystems.forEach(s -> s.zeroSensors());
        pauseLoops = false;
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }

    private class EnabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            mLoops.forEach(l -> l.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            if (!pauseLoops) {

                mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
                mLoops.forEach(l -> l.onLoop(timestamp));
                mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
            }
        }

        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
        }

        @Override
        public void onLoop(double timestamp) {
            if (!pauseLoops) mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
        }

        @Override
        public void onStop(double timestamp) {
        }
    }
}
