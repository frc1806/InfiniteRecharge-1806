package com.team1806.frc2020.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1806.frc2020.Constants;
import com.team1806.frc2020.controlboard.ControlBoard;
import com.team1806.lib.util.CircularBuffer;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Hood extends Subsystem {

    private static Hood HOOD = new Hood();
    private HoodControlState mHoodControlState;
    private TalonSRX mCANTalonSRX;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
    private boolean mIsAngleValid;
    private CircularBuffer ampsBuffer = new CircularBuffer(10);
    private Hood() {
        mHoodControlState = HoodControlState.kIdle;

        mCANTalonSRX = new TalonSRX(Constants.kHoodMotorId);
        mPeriodicIO = new PeriodicIO();
        mPeriodicIO.HoodState = HoodControlState.kIdle;
        mHoodControlState = HoodControlState.kIdle;
        mCANTalonSRX.setNeutralMode(NeutralMode.Coast);
        mIsAngleValid = false;
        reloadGains();
        mCANTalonSRX.setInverted(true);
        mCANTalonSRX.setSensorPhase(false);
    }

    public static Hood GetInstance() {
        return HOOD;
    }

    private double ConvertEncoderClicksToAngle(double encoderCount) {
        return encoderCount * Constants.kHoodDegreesPerCount;
    }

    private double ConvertAngleToEncoderClicks(double angle) {
        return angle / Constants.kHoodDegreesPerCount;
    }

    public void setWantedAngle(double angle) {
        setControlState(HoodControlState.kPositionControl);
        if (mPeriodicIO.wantedAngle <= Constants.kHoodPositionMax && mPeriodicIO.wantedAngle >= Constants.kHoodPositionMin) {
            mIsAngleValid = true;
            mPeriodicIO.wantedAngle = angle;
        } else if (mPeriodicIO.wantedAngle > Constants.kHoodPositionMax) {
            mIsAngleValid = false;
            mPeriodicIO.wantedAngle = Constants.kHoodPositionMax;
        } else {
            mIsAngleValid = false;
            mPeriodicIO.wantedAngle = Constants.kHoodPositionMin;
        }
        if(mPeriodicIO.currentAngle > 2.0 && mPeriodicIO.wantedAngle <=0.0 ){
            setWantRetract();
        }
    }

    public void setManualControl(double wantedPosition) {
        setControlState(HoodControlState.kManualControl);
        mPeriodicIO.wantedManualMovement = wantedPosition;
    }

    public void setWantIdle() {
        setControlState(HoodControlState.kIdle);
        mPeriodicIO.wantedAngle = 0.0;
        mPeriodicIO.wantedManualMovement = 0.0;
    }

    public boolean isOnTarget() {
        return (Math.abs(mPeriodicIO.wantedAngle - mPeriodicIO.currentAngle) < Constants.kHoodAcceptableAngleDeviation && mIsAngleValid)
                || ControlBoard.GetInstance().getWantManualHood();
    }

    private void reloadGains() {
        if (mCANTalonSRX != null) {
            mCANTalonSRX.config_kP(0, Constants.kHoodPositionControlKp);
            mCANTalonSRX.config_kI(0, Constants.kHoodPositionControlKi);
            mCANTalonSRX.config_kD(0, Constants.kHoodPositionControlKd);
            mCANTalonSRX.config_kF(0, Constants.kHoodPositionControlKf);
        }
    }

    public void writePeriodicOutputs() {
        if (mPeriodicIO.currentAngle <= Constants.kHoodPositionMin - 8 && mHoodControlState != HoodControlState.kManualControl) {
            mCANTalonSRX.set(ControlMode.PercentOutput, .2);
        } else if (mPeriodicIO.currentAngle >= Constants.kHoodPositionMax + 5 && mHoodControlState != HoodControlState.kManualControl) {
            mCANTalonSRX.set(ControlMode.PercentOutput, -.2);
        } else if(ampsBuffer.getAverage() > Constants.kHoodMaxAmpLimit) {
            setControlState(HoodControlState.kDisabled);
        }else{
            switch (mPeriodicIO.HoodState) {
                default:
                case kIdle:
                case kDisabled:
                    mCANTalonSRX.set(ControlMode.PercentOutput, 0);
                    break;
                case kPositionControl:
                    if (isOnTarget()) {
                        setControlState(HoodControlState.kHoldPosition);
                    } else {
                        mCANTalonSRX.set(ControlMode.Position, ConvertAngleToEncoderClicks(mPeriodicIO.wantedAngle));
                    }
                    break;
                case kHoldPosition:
                    mCANTalonSRX.set(ControlMode.PercentOutput, 0);

                    if (!isOnTarget()) {
                        setControlState(HoodControlState.kPositionControl);
                    } else {
                        mCANTalonSRX.set(ControlMode.PercentOutput, 0);

                    }

                    break;
                case kManualControl:
                    mCANTalonSRX.set(ControlMode.Position, mPeriodicIO.wantedManualMovement);
                    break;

                case kRetract:
                    mCANTalonSRX.set(ControlMode.Position, ConvertAngleToEncoderClicks(-9.0));
                    if (mPeriodicIO.currentAngle < -7.5) {
                        setWantedAngle(0.0);
                    }
                    break;
            }
        }

    }

    public void readPeriodicInputs() {
        mPeriodicIO.motorVoltage = mCANTalonSRX.getMotorOutputVoltage();
        mPeriodicIO.motorAmps = mCANTalonSRX.getStatorCurrent();
        double lastPosition = mPeriodicIO.currentAngle;
        mPeriodicIO.currentEncoderClicks = mCANTalonSRX.getSelectedSensorPosition();
        mPeriodicIO.currentAngle = ConvertEncoderClicksToAngle(mCANTalonSRX.getSelectedSensorPosition());
        double lastTimestamp = mPeriodicIO.timestamp;
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.currentSpeed = mCANTalonSRX.getSelectedSensorVelocity();
        ampsBuffer.addValue(mPeriodicIO.motorAmps);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    public void zeroSensors() {
        mCANTalonSRX.setSelectedSensorPosition(0);
    }

    public void setControlState(HoodControlState state) {
        if(mHoodControlState != HoodControlState.kDisabled || state == HoodControlState.kIdle){
            mPeriodicIO.HoodState = state;
            mHoodControlState = state;
        }
    }

    public void stop() {
        setControlState(HoodControlState.kIdle);
    }

    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Angle", mPeriodicIO.currentAngle);
        SmartDashboard.putNumber("Current Hood Encoder Clicks", mPeriodicIO.currentEncoderClicks);
        SmartDashboard.putString("Hood Control State", mPeriodicIO.HoodState.toString());
        SmartDashboard.putNumber("Current Hood Motor AMPS", mPeriodicIO.motorAmps);
        SmartDashboard.putNumber("Smoothed Hood Motor AMPS", ampsBuffer.getAverage());


        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/HOOD-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public synchronized void setWantRetract() {
        setControlState(HoodControlState.kRetract);
    }

    public synchronized boolean isRetracting() {
        return mHoodControlState == HoodControlState.kRetract;
    }

    public synchronized void enableHood(){
        setControlState(HoodControlState.kIdle);
    }

    enum HoodControlState {

        kIdle, kPositionControl, kHoldPosition, kManualControl, kRetract, kDisabled
    }

    private class PeriodicIO {
        public double timestamp;
        public double motorVoltage;
        public double motorAmps;
        public double currentAngle;
        public double currentSpeed;
        public double wantedAngle;
        public double wantedManualMovement;
        public HoodControlState HoodState;
        public int currentEncoderClicks;


    }
}
