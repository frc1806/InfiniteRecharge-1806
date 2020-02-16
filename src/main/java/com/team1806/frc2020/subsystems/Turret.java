package com.team1806.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1806.frc2020.Constants;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Turret extends Subsystem {

    enum TurretControlState {

        kIdle, kPositionControl, kHoldPosition, kManualControl
    }


    private class PeriodicIO {
        public double timestamp;
        public double motorVoltage;
        public double motorAmps;
        public double currentAngle;
        public double currentSpeed;
        public double wantedAngle;
        public double wantedManualMovement;
        public TurretControlState turretState;

    }



    private static Turret TURRET = new Turret();

    private TurretControlState mTurretControlState;
    private TalonSRX mCANTalonSRX;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;


    private Turret(){
        mCANTalonSRX = new TalonSRX(Constants.kTurretMotorId);
        mPeriodicIO = new PeriodicIO();
        mTurretControlState = TurretControlState.kIdle;
        mCANTalonSRX.setNeutralMode(NeutralMode.Brake);
        reloadGains();
    }

    public static Turret GetInstance(){return TURRET;}

    private double ConvertEncoderClicksToAngle(double encoderCount){
        return encoderCount * Constants.kTurretDegreesPerCount;
    }

    private double ConvertAngleToEncoderClicks(double angle){
        return angle / Constants.kTurretDegreesPerCount;
    }

    public void setWantedAngle(double angle){
        setControlState(TurretControlState.kPositionControl);
        mPeriodicIO.wantedAngle = angle;
    }

    public void setManualControl(double wantedPosition){
        setControlState(TurretControlState.kManualControl);
        mPeriodicIO.wantedManualMovement = wantedPosition;
    }

    public void setWantIdle(){
        setControlState(TurretControlState.kIdle);
        mPeriodicIO.wantedAngle = 0.0;
        mPeriodicIO.wantedManualMovement = 0.0;
    }

    public boolean isOnTarget() {
        return Math.abs(mPeriodicIO.wantedAngle - mPeriodicIO.currentAngle) < Constants.kTurretAcceptableAngleDeviation && Math.abs(mPeriodicIO.currentSpeed) <= Constants.kTurretAcceptableSpeed;
    }

    private void reloadGains(){
        if(mCANTalonSRX!= null){
            mCANTalonSRX.config_kP(0, Constants.kTurretPositionControlKp);
            mCANTalonSRX.config_kI(0, Constants.kTurretPositionControlKi);
            mCANTalonSRX.config_kD(0,Constants.kTurretPositionControlKd);
            mCANTalonSRX.config_kF(0, Constants.kTurretPositionControlKf);
        }
    }

    public void writePeriodicOutputs(){
        switch (mPeriodicIO.turretState){
            default:
            case kIdle:
                mCANTalonSRX.set(ControlMode.PercentOutput, 0);
                break;
            case kPositionControl:
                if(isOnTarget()){
                    setControlState(TurretControlState.kHoldPosition);
                }
                else{
                    mCANTalonSRX.set(ControlMode.Position, ConvertAngleToEncoderClicks(mPeriodicIO.wantedAngle));
                }
                break;
            case kHoldPosition:
                mCANTalonSRX.set(ControlMode.PercentOutput, 0);

                if(!isOnTarget()){
                    setControlState(TurretControlState.kPositionControl);
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
        mPeriodicIO.motorAmps = mCANTalonSRX.getStatorCurrent();
        double lastPosition = mPeriodicIO.currentAngle;
        mPeriodicIO.currentAngle = ConvertEncoderClicksToAngle(mCANTalonSRX.getSelectedSensorPosition());
        double lastTimestamp = mPeriodicIO.timestamp;
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.currentSpeed = mCANTalonSRX.getSelectedSensorVelocity();


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
        SmartDashboard.putNumber("Turret Angle", mPeriodicIO.currentAngle);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

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
