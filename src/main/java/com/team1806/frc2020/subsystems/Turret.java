package com.team1806.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1806.frc2020.Constants;
import com.team1806.frc2020.RobotState;
import com.team1806.frc2020.controlboard.ControlBoard;
import com.team1806.lib.geometry.Rotation2d;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.naming.ldap.Control;


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

        public int currentEncoderClicks;

    }



    private static Turret TURRET = new Turret();

    private TurretControlState mTurretControlState;
    private TalonSRX mCANTalonSRX;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
    public boolean mIsAngleValid;


    private Turret(){
        mCANTalonSRX = new TalonSRX(Constants.kTurretMotorId);
        mPeriodicIO = new PeriodicIO();
        mTurretControlState = TurretControlState.kIdle;
        mCANTalonSRX.setNeutralMode(NeutralMode.Brake);
        mIsAngleValid = false;
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
        if (mPeriodicIO.wantedAngle <= Constants.kTurretPositionMax && mPeriodicIO.wantedAngle >= Constants.kTurretPositionMin) {
            mIsAngleValid = true;
            mPeriodicIO.wantedAngle = angle;
        }
        else if (mPeriodicIO.wantedAngle > Constants.kTurretPositionMax){
            mIsAngleValid = false;
            mPeriodicIO.wantedAngle = Constants.kTurretPositionMax;
        }
        else {
            mIsAngleValid = false;
            mPeriodicIO.wantedAngle = Constants.kTurretPositionMin;
        }
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
        return (Math.abs(mPeriodicIO.wantedAngle - mPeriodicIO.currentAngle) <= Constants.kTurretAcceptableAngleDeviation && Math.abs(mPeriodicIO.currentSpeed) <= Constants.kTurretAcceptableSpeed && mIsAngleValid)
                || ControlBoard.GetInstance().getWantManualTurret();
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
        switch (mPeriodicIO.turretState) {
            default:
            case kIdle:
                mCANTalonSRX.set(ControlMode.PercentOutput, 0);
                break;
            case kPositionControl:
                if (isOnTarget()) {
                    setControlState(TurretControlState.kHoldPosition);
                } else {
                    mCANTalonSRX.set(ControlMode.Position, ConvertAngleToEncoderClicks(mPeriodicIO.wantedAngle));
                }
                break;
            case kHoldPosition:
                mCANTalonSRX.set(ControlMode.PercentOutput, 0);

                if (!isOnTarget()) {
                    setControlState(TurretControlState.kPositionControl);
                } else {
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
        mPeriodicIO.currentEncoderClicks = mCANTalonSRX.getSelectedSensorPosition();
        mPeriodicIO.currentAngle = ConvertEncoderClicksToAngle(mCANTalonSRX.getSelectedSensorPosition());
        double lastTimestamp = mPeriodicIO.timestamp;
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.currentSpeed = mCANTalonSRX.getSelectedSensorVelocity();
        RobotState.getInstance().addVehicleToTurretObservation(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(mPeriodicIO.currentAngle));


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
        SmartDashboard.putNumber("Current Turret Encoder Clicks", mPeriodicIO.currentEncoderClicks);

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

    private void addTurretObservation(){

    }

    public Rotation2d getCurrentAngle(){
        return Rotation2d.fromDegrees(mPeriodicIO.currentAngle);
    }

    public Rotation2d getWantedAngle(){
        return Rotation2d.fromDegrees(mPeriodicIO.wantedAngle);
    }


}
