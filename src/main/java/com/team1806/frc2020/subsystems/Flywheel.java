package com.team1806.frc2020.subsystems;


import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.team1806.frc2020.Constants;
import com.team1806.lib.drivers.LazySparkMax;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Flywheel extends Subsystem {


    public static Flywheel FLYWHEEL = new Flywheel();
    private LazySparkMax mSparkMaxLeader;
    private LazySparkMax mSparkMaxFollower;
    private FlywheelControlState mFlywheelControlState;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
    private Flywheel() {
        mSparkMaxLeader = new LazySparkMax(Constants.kFlywheelSparkMaxLeader);
        mSparkMaxFollower = new LazySparkMax(Constants.kFlywheelSparkMaxFollower);
        mSparkMaxLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mSparkMaxFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mSparkMaxLeader.setInverted(true);
        mSparkMaxFollower.setInverted(true);
        mSparkMaxFollower.follow(mSparkMaxLeader);
        mPeriodicIO = new PeriodicIO();
        setControlState(FlywheelControlState.kIdle);
        mSparkMaxLeader.setSmartCurrentLimit(Constants.kFlywheelSmartCurrentLimit);
        mSparkMaxFollower.setSmartCurrentLimit(Constants.kFlywheelSmartCurrentLimit);
        mSparkMaxLeader.burnFlash();
        mSparkMaxFollower.burnFlash();
        reloadGains();

    }

    public static Flywheel GetInstance() {
        return FLYWHEEL;
    }

    public void setSpeed(double RPM) {
        setControlState(FlywheelControlState.kSpeedControlled);
        mPeriodicIO.wantedRPM = RPM;
    }

    public void setManualControl(double power) {
        mPeriodicIO.wantedDemand = power;
        setControlState(FlywheelControlState.kManualControlled);
    }

    public void setWantIdle() {
        mPeriodicIO.wantedRPM = 0.0;
        setControlState(FlywheelControlState.kIdle);
        mPeriodicIO.wantedDemand = 0.0;
    }

    public double getDesiredSpeed() {
        return mPeriodicIO.wantedRPM;
    }

    private double convertNEOSpeedtoLauncherSpeed(double NEORPM) {
        return NEORPM * Constants.kFlywheelGearScalingFactor;
    }

    private double convertLauncherSpeedToNEOSpeed(double launcherRPM) {
        return launcherRPM / Constants.kFlywheelGearScalingFactor;
    }

    public boolean isReadyForLaunch() {
        return Math.abs(mPeriodicIO.launchWheelRPM - mPeriodicIO.wantedRPM) < Constants.kFlywheelAcceptableSpeedRange; //&& Math.abs(mPeriodicIO.launchWheelAccel) < Constants.kFlywheelAcceptableAccleration;
    }

    /**
     * Sets the control state both on mFlywheelControlState and periodic io so the states are logged.
     *
     * @param state the desired control state
     */
    private void setControlState(FlywheelControlState state) {
        mPeriodicIO.currentMode = state;
        mFlywheelControlState = state;
    }

    private void reloadGains() {
        if (mSparkMaxLeader != null && mSparkMaxLeader.getPIDController() != null) {
            ArrayList<CANError> errors = new ArrayList<>();
            errors.add(mSparkMaxLeader.getPIDController().setP(Constants.kFlywheelSpeedControlkp));
            errors.add(mSparkMaxLeader.getPIDController().setI(Constants.kFlywheelSpeedControlki));
            errors.add(mSparkMaxLeader.getPIDController().setD(Constants.kFlywheelSpeedControlkd));
            errors.add(mSparkMaxLeader.getPIDController().setFF(Constants.kFlywheelSpeedControlkf, 0));
            errors.add(mSparkMaxLeader.getPIDController().setOutputRange(0, 1));
            System.out.println(errors);
        }
    }

    public void writeToLog() {


    }

    public void readPeriodicInputs() {
        double lastTimestamp = mPeriodicIO.timestamp;
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.leaderVoltage = mSparkMaxLeader.getBusVoltage() * mSparkMaxLeader.getAppliedOutput();
        mPeriodicIO.followerVoltage = mSparkMaxFollower.getBusVoltage() * mSparkMaxFollower.getAppliedOutput();
        mPeriodicIO.leaderCurrent = mSparkMaxLeader.getOutputCurrent();
        mPeriodicIO.followerCurrent = mSparkMaxFollower.getOutputCurrent();
        double lastVelocity = mPeriodicIO.launchWheelRPM;
        mPeriodicIO.launchWheelRPM = convertNEOSpeedtoLauncherSpeed(mSparkMaxLeader.getEncoder().getVelocity());


        mPeriodicIO.launchWheelAccel = (mPeriodicIO.launchWheelRPM - lastVelocity) / (mPeriodicIO.timestamp - lastTimestamp);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

    }

    public void writePeriodicOutputs() {
        switch (mPeriodicIO.currentMode) {
            default:
            case kIdle:
                mSparkMaxLeader.set(0);

                break;
            case kSpeedControlled:
                mSparkMaxLeader.getPIDController().setReference(convertLauncherSpeedToNEOSpeed(mPeriodicIO.wantedRPM), ControlType.kVelocity);
                break;
            case kManualControlled:

                mSparkMaxLeader.set(mPeriodicIO.wantedDemand);

                break;
        }


    }

    public void stop() {
        mPeriodicIO.wantedRPM = 0;
        setControlState(FlywheelControlState.kIdle);

    }

    public boolean checkSystem() {
        return true;

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Launcher Wheel RPM", mPeriodicIO.launchWheelRPM);
        SmartDashboard.putNumber("Launcher Wheel Accel", mPeriodicIO.launchWheelAccel);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/FLYWHEEL-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    enum FlywheelControlState {
        kIdle, kSpeedControlled, kManualControlled
    }

    private class PeriodicIO {
        public double timestamp;

        //inputs read from robot
        public double leaderVoltage;
        public double followerVoltage;
        public double leaderCurrent;
        public double followerCurrent;
        public double launchWheelRPM;

        //outputs set to robot
        public double launchWheelAccel;
        public double wantedDemand;
        public double wantedRPM;
        public FlywheelControlState currentMode;
    }

}
