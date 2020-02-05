package com.team1806.frc2020.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1806.frc2020.Constants;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Hood extends Subsystem {

    enum HoodControlState {

        kIdle, kPositionControl, kHoldPosition, kManualControl
    }


    private class PeriodicIO {
        public double timestamp;
        public double motorVoltage;
        public double motorAmps;
        public double currentPosition;
        public double currentSpeed;
        public double wantedPosition;
        public double wantedManualMovement;
        public HoodControlState HoodState;

    }



    private static Hood HOOD = new Hood();

    private HoodControlState mHoodControlState;
    private TalonSRX mCANTalonSRX;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter mCSVWriter;


    private Hood(){
        mCANTalonSRX = new TalonSRX(Constants.kHoodMotor);
        mPeriodicIO = new PeriodicIO();
        mHoodControlState = HoodControlState.kIdle;
        mCANTalonSRX.setNeutralMode(NeutralMode.Brake);
    }

    public static Hood GetInstance(){
        return HOOD;
    }

    private double ConvertEncoderClicksToAngle(double encoderCount){
        return encoderCount * Constants.kHoodDegreesPerCount;
    }

    private double ConvertAngleToEncoderClicks(double angle){
        return angle / Constants.kHoodDegreesPerCount;
    }

    public void setWantedAngle(double angle){
        setControlState(HoodControlState.kPositionControl);
        mPeriodicIO.wantedPosition = angle;
    }

    public void setManualControl(double wantedPosition){
        setControlState(HoodControlState.kManualControl);
        mPeriodicIO.wantedManualMovement = wantedPosition;
    }

    public void setWantIdle(){
        setControlState(HoodControlState.kIdle);
        mPeriodicIO.wantedPosition = 0.0;
        mPeriodicIO.wantedManualMovement = 0.0;
    }

    public boolean isOnTarget() {
        return Math.abs(mPeriodicIO.wantedPosition - mPeriodicIO.currentPosition) < Constants.kHoodAcceptableAngleDeviation && Math.abs(mPeriodicIO.currentSpeed) <= Constants.kHoodAcceptableSpeed;
    }

    private void reloadGains(){
        if(mCANTalonSRX!= null){
            mCANTalonSRX.config_kP(0, Constants.kHoodPositionControlKp);
            mCANTalonSRX.config_kI(0, Constants.kHoodPositionControlKi);
            mCANTalonSRX.config_kD(0,Constants.kHoodPositionControlKd);
            mCANTalonSRX.config_kF(0, Constants.kHoodPositionControlKf);
        }
    }

    public void writePeriodicOutputs(){
        switch (mPeriodicIO.HoodState){
            default:
            case kIdle:
                mCANTalonSRX.set(ControlMode.PercentOutput, 0);
                break;
            case kPositionControl:
                if(isOnTarget()){
                    setControlState(HoodControlState.kHoldPosition);
                }
                else{
                    mCANTalonSRX.set(ControlMode.Position, mPeriodicIO.wantedPosition);
                }
                break;
            case kHoldPosition:
                mCANTalonSRX.set(ControlMode.PercentOutput, 0);

                if(!isOnTarget()){
                    setControlState(HoodControlState.kPositionControl);
                }

                else{
                    mCANTalonSRX.set(ControlMode.PercentOutput, 0);

                }

                break;
            case kManualControl:
                mCANTalonSRX.set(ControlMode.Position, mPeriodicIO.wantedManualMovement);
                break;
        }
    }

    public void readPeriodicInputs(){
        mPeriodicIO.motorVoltage = mCANTalonSRX.getMotorOutputVoltage();
        mPeriodicIO.motorAmps = mCANTalonSRX.getStatorCurrent(); //There is also a getSupplyCurrent if that is what we needed to use instead
        double lastPosition = mPeriodicIO.currentPosition;
        mPeriodicIO.currentPosition = ConvertEncoderClicksToAngle(mCANTalonSRX.getSelectedSensorPosition()); //Optional arg for "pidIdx" Do we need it?
        double lastTimestamp = mPeriodicIO.timestamp;
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.currentSpeed = mPeriodicIO.wantedPosition - lastPosition / mPeriodicIO.timestamp - lastTimestamp;

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }



    public void zeroSensors(){
        mCANTalonSRX.setSelectedSensorPosition(0);
    }

    public void setControlState(HoodControlState state) {
        mPeriodicIO.HoodState = state;
        mHoodControlState = state;

    }

    public void stop(){
        setControlState(HoodControlState.kIdle);
    }

    public boolean checkSystem(){
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Angle", mPeriodicIO.currentPosition);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

    }

    public synchronized void startLogging(){
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/HOOD-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging(){
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

}
