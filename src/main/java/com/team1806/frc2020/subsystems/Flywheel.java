package com.team1806.frc2020.subsystems;


import com.revrobotics.ControlType;
import com.team1806.frc2020.Constants;
import com.team1806.frc2020.loops.ILooper;
import com.team1806.lib.drivers.LazySparkMax;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends Subsystem {


    enum FlywheelControlState {
        kIdle, kSpeedControlled, kManualControlled;
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

    public static Flywheel FLYWHEEL = new Flywheel();
    private LazySparkMax mSparkMaxLeader;
    private LazySparkMax mSparkMaxFollower;
    private double mDesiredLauncherWheelSpeed;
    private FlywheelControlState mFlywheelControlState;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter mCSVWriter;

    private Flywheel(){

        LazySparkMax mSparkMaxLeader = new LazySparkMax(30);
        LazySparkMax mSparkMaxFollower = new LazySparkMax(31);

        mPeriodicIO = new PeriodicIO();
        setControlState(FlywheelControlState.kIdle);
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

    private double convertLauncherSpeedToNEOSpeed(double launcherRPM){
        return launcherRPM / Constants.kFlywheelGearScalingFactor;
    }

    public boolean isReadyForLaunch() {
        if (mPeriodicIO.launchWheelRPM < mPeriodicIO.wantedRPM + 100 && mPeriodicIO.launchWheelRPM > mPeriodicIO.wantedRPM - 100) {
            return true;
        }
        return false;
    }

    /**
     * Sets the control state both on mFlywheelControlState and periodic io so the states are logged.
     * @param state the desired control state
     */
    private void setControlState(FlywheelControlState state){
        mPeriodicIO.currentMode = state;
        mFlywheelControlState = state;
    }

    private void reloadGains() {
        mSparkMaxLeader.getPIDController().setP(Constants.kFlywheelSpeedControlkp);
        mSparkMaxLeader.getPIDController().setI(Constants.kFlywheelSpeedControlki);
        mSparkMaxLeader.getPIDController().setD(Constants.kFlywheelSpeedControlkd);
    }

    public void registerEnabledLoops(ILooper mEnabledLooper){

    }

    public void writeToLog(){


    }

    public void readPeriodicInputs(){
        double lastTimestamp = mPeriodicIO.timestamp;
       mPeriodicIO.timestamp = Timer.getFPGATimestamp();
       mPeriodicIO.leaderVoltage = mSparkMaxLeader.getBusVoltage() * mSparkMaxLeader.getAppliedOutput();
       mPeriodicIO.followerVoltage = mSparkMaxFollower.getBusVoltage() * mSparkMaxFollower.getAppliedOutput();
       mPeriodicIO.leaderCurrent = mSparkMaxLeader.getOutputCurrent();
       mPeriodicIO.followerCurrent = mSparkMaxFollower.getOutputCurrent();
       double lastVelocity = mPeriodicIO.launchWheelRPM;
       mPeriodicIO.launchWheelRPM = convertLauncherSpeedToNEOSpeed(mSparkMaxLeader.getEncoder().getVelocity());




       mPeriodicIO.launchWheelAccel = mPeriodicIO.timestamp - lastTimestamp / mPeriodicIO.launchWheelRPM - lastVelocity;

       if (mCSVWriter != null) {
           mCSVWriter.add(mPeriodicIO);
       }

    }

    public void writePeriodicOutputs() {
        switch(mPeriodicIO.currentMode){
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
        mDesiredLauncherWheelSpeed = 0;
        setControlState(FlywheelControlState.kIdle);

    }

    public boolean checkSystem(){
        return true;

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Launcher Wheel RPM", mPeriodicIO.launchWheelRPM);

         if(mCSVWriter != null){
             mCSVWriter.write();
         }


    }

}
