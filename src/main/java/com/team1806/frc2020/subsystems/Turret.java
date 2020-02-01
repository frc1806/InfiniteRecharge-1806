package com.team1806.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1806.frc2020.Constants;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Turret extends Subsystem {

    enum TurretControlState {

        kIdle, kPositionControl, kHoldPosition, kManualControl;
    }


    private class PeriodicIO {
        public double timestamp;
        public double motorVoltage;
        public double motorAmps;
        public double currentPosition;
        public double currentSpeed;
        public double wantedPosition;
        public double wantedManualMovement;
        public TurretControlState turretState;

    }



    private static Turret TURRET = new Turret();

    private TurretControlState mTurretControlState;
    private TalonSRX mCANTalonSRX;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter mCSVWriter;


    private Turret(){
        mCANTalonSRX = new TalonSRX(Constants.kTurretMotor);
        mPeriodicIO = new PeriodicIO();
    }

    public static Turret GetInstance(){
        return TURRET;
    }

    private double ConvertEncoderClicksToAngle(double encoderCount){
        return encoderCount * Constants.kTurretDegreesPerCount;
    }

    private double ConvertAngleToEncoderClicks(double angle){
        return angle / Constants.kTurretDegreesPerCount;
    }

    public void setWantedAngle(double angle){
        setControlState(TurretControlState.kPositionControl);
        mPeriodicIO.wantedPosition = angle;
    }

    public void setManualControl(double wantedPosition){
        setControlState(TurretControlState.kManualControl);
        mPeriodicIO.wantedManualMovement = wantedPosition;
    }

    public void setWantIdle(){
        setControlState(TurretControlState.kIdle);
        mPeriodicIO.wantedPosition = 0.0;
        mPeriodicIO.wantedManualMovement = 0.0;
    }

    public boolean isOnTarget() {
        return Math.abs(mPeriodicIO.currentPosition) >= Math.abs(mPeriodicIO.wantedPosition - Constants.kTurretAcceptableAngleDeviation) && mPeriodicIO.currentSpeed <= Constants.kTurretAcceptableSpeed;
    }



    public void writePeriodicOutputs(){
        switch (mPeriodicIO.turretState){
            default:
            case kIdle:
                mCANTalonSRX.set(ControlMode.Position, 0);
                break;
            case kPositionControl:
                mCANTalonSRX.set(ControlMode.Position, mPeriodicIO.wantedPosition);
                break;
            case kHoldPosition:
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

    public void setControlState(TurretControlState state) {
        mPeriodicIO.turretState = state;
        mTurretControlState = state;

    }

    public void stop(){
        setControlState(TurretControlState.kIdle);
    }

    public boolean checkSystem(){
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Turret Angle", mPeriodicIO.currentPosition);
        mCSVWriter.write();

    }

    public synchronized void startLogging(){
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/TURRET-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging(){
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

}
