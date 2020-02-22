package com.team1806.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.team1806.frc2020.Constants;
import com.team1806.lib.drivers.LazySparkMax;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Conveyor extends Subsystem {

    private final boolean SPEED_CONTROL_TRIGGER = false;
    private final boolean SPEED_CONTROL_UPPER = false;
    private final boolean SPEED_CONTROL_BOTTOM = false;

    enum ConveyorControlState {

        kIdle, kFront, kBack, kLaunching, kPositionalControl, kRotationalControl
    }


    private class PeriodicIO {
        public double currentTimestamp;

        public double triggerMotorVoltage;
        public double topMotorVoltage;
        public double bottomMotorVoltage;

        public double triggerMotorAmps;
        public double topMotorAmps;
        public double bottomMotorAmps;

        public double triggerCurrentVelocity;
        public double topCurrentVelocity;
        public double bottomCurrentVelocity;

        public double distance;
        public boolean isEmpty;
        public boolean wasEmpty;
        public boolean isFull;
        public boolean wasFull;
        public double timeEmptied;
        public double timeFilled;
        public boolean isDoneShooting;
        public boolean isTopDoneConveyoring;

        public boolean frontIsExtended;
        public boolean backIsExtended;
        public ColorWheelReader.MatchedColor wantedColor;
        public String FMS_Color;

        public ConveyorControlState ConveyorState;

        public ConveyorControlState lastIntakeDirection;

        public boolean isReadyToLaunch;

    }


    private static Conveyor CONVEYOR = new Conveyor();

    private ConveyorControlState mConveyorControlState;
    private TalonSRX mTriggerCANTalonSRX, mTopCANTalonSRX, mBottomCANTalonSRX;
    private LazySparkMax mOuterIntakeSparkMAX;
    private DoubleSolenoid mFrontSolenoid, mBackSolenoid;
    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
    private ConveyorControlState mLastIntakeDirection;
    private ColorWheelReader mColorWheelReader;
    private Rev2mDistanceSensor mDistanceSensor;



    private Conveyor(){
        mPeriodicIO = new PeriodicIO();
        mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kUnknown;

        mTriggerCANTalonSRX = new TalonSRX(Constants.kTriggerConveyorMotorId);
        mTopCANTalonSRX = new TalonSRX(Constants.kTopConveyorMotorId);
        mBottomCANTalonSRX = new TalonSRX(Constants.kBottomConveyorMotorId);
        mOuterIntakeSparkMAX = new LazySparkMax(Constants.kOuterIntake);

        mFrontSolenoid = new DoubleSolenoid(Constants.kFrontIntakeFowardChannel, Constants.kFrontIntakeReverseChannel);
        mBackSolenoid = new DoubleSolenoid(Constants.kBackIntakeFowardChannel, Constants.kBackIntakeReverseChannel);

        mDistanceSensor = new Rev2mDistanceSensor(Port.kMXP);
        mDistanceSensor.setAutomaticMode(true);
        mDistanceSensor.setEnabled(true);
        mDistanceSensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kInches);
        mDistanceSensor.setRangeProfile(RangeProfile.kHighSpeed);

        setControlState(ConveyorControlState.kIdle);
        mLastIntakeDirection = ConveyorControlState.kFront;

        mTriggerCANTalonSRX.setNeutralMode(NeutralMode.Brake);
        mTopCANTalonSRX.setNeutralMode(NeutralMode.Brake);
        mBottomCANTalonSRX.setNeutralMode(NeutralMode.Brake);

        mOuterIntakeSparkMAX.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mOuterIntakeSparkMAX.setSmartCurrentLimit(Constants.kOuterIntakeSmartCurrentLimit);
        mOuterIntakeSparkMAX.burnFlash();

        mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kUnknown;

        mTriggerCANTalonSRX.setInverted(true);
        mBottomCANTalonSRX.setInverted(true);


        triggerReloadGains();
        topReloadGains();
        bottomReloadGains();
        outerIntakeReloadGains();

        mColorWheelReader = ColorWheelReader.GetInstance();
    }

    public static Conveyor GetInstance(){
        return CONVEYOR;
    }


    private void triggerReloadGains(){
        if(mTriggerCANTalonSRX!= null){
            mTriggerCANTalonSRX.config_kP(0, Constants.kTriggerConveyorVelocityControlKp);
            mTriggerCANTalonSRX.config_kI(0, Constants.kTriggerConveyorVelocityControlKi);
            mTriggerCANTalonSRX.config_kD(0,Constants.kTriggerConveyorVelocityControlKd);
            mTriggerCANTalonSRX.config_kF(0, Constants.kTriggerConveyorVelocityControlKf);
        }
    }

    private void topReloadGains(){
        if(mTopCANTalonSRX!= null){
            mTopCANTalonSRX.config_kP(0, Constants.kTopConveyorVelocityControlKp);
            mTopCANTalonSRX.config_kI(0, Constants.kTopConveyorVelocityControlKi);
            mTopCANTalonSRX.config_kD(0,Constants.kTopConveyorVelocityControlKd);
            mTopCANTalonSRX.config_kF(0, Constants.kTopConveyorVelocityControlKf);
        }
    }

    private void bottomReloadGains(){
        if(mBottomCANTalonSRX!= null){
            mBottomCANTalonSRX.config_kP(0, Constants.kBottomConveyorVelocityControlKp);
            mBottomCANTalonSRX.config_kI(0, Constants.kBottomConveyorVelocityControlKi);
            mBottomCANTalonSRX.config_kD(0,Constants.kBottomConveyorVelocityControlKd);
            mBottomCANTalonSRX.config_kF(0, Constants.kBottomConveyorVelocityControlKf);
        }
    }

    private void outerIntakeReloadGains(){
        if(mOuterIntakeSparkMAX!= null && mOuterIntakeSparkMAX.getPIDController() != null){
            mOuterIntakeSparkMAX.getPIDController().setP(Constants.kOuterIntakeVelocityControlKp);
            mOuterIntakeSparkMAX.getPIDController().setI(Constants.kOuterIntakeVelocityControlKi);
            mOuterIntakeSparkMAX.getPIDController().setD(Constants.kOuterIntakeVelocityControlKd);
            mOuterIntakeSparkMAX.getPIDController().setFF(Constants.kOuterIntakeVelocityControlKf);

        }
    }

    public void writePeriodicOutputs(){
        switch (mPeriodicIO.ConveyorState) {
            default:
            case kIdle:
                mColorWheelReader.stopSensing();
                mTriggerCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mBottomCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mFrontSolenoid.set(DoubleSolenoid.Value.kReverse);
                mBackSolenoid.set(DoubleSolenoid.Value.kReverse);

                break;
            case kFront:
                if (!mPeriodicIO.frontIsExtended) {
                    mFrontSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                mBackSolenoid.set(DoubleSolenoid.Value.kReverse);
                mOuterIntakeSparkMAX.getPIDController().setReference(Constants.kOuterIntakeSpeed, ControlType.kVelocity);
                if (!mPeriodicIO.isTopDoneConveyoring){
                    if(SPEED_CONTROL_UPPER){
                        mTopCANTalonSRX.set(ControlMode.Velocity, Constants.kTopConveyorSpeed);
                    }
                    else{
                        mTopCANTalonSRX.set(ControlMode.PercentOutput, Constants.kTopConveyorDutyCycle);
                    }

                } else {
                    mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                }
                if(SPEED_CONTROL_BOTTOM){
                    mBottomCANTalonSRX.set(ControlMode.Velocity, Constants.kBottomConveyorSpeed);
                }
                else{
                    mBottomCANTalonSRX.set(ControlMode.PercentOutput, Constants.kBottomConveyorDutyCycle);
                }

                break;
            case kBack:
                if (!mPeriodicIO.backIsExtended) {
                    mBackSolenoid.set(DoubleSolenoid.Value.kForward);
                }
                mFrontSolenoid.set(DoubleSolenoid.Value.kReverse);
                mOuterIntakeSparkMAX.getPIDController().setReference(-Constants.kOuterIntakeSpeed, ControlType.kVelocity);
                if (!mPeriodicIO.isTopDoneConveyoring){
                    if(SPEED_CONTROL_UPPER){
                        mTopCANTalonSRX.set(ControlMode.Velocity, Constants.kTopConveyorSpeed);
                    }
                    else{
                        mTopCANTalonSRX.set(ControlMode.PercentOutput, Constants.kTopConveyorDutyCycle);
                    }
                } else {
                    mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                }
                if(SPEED_CONTROL_BOTTOM){
                    mBottomCANTalonSRX.set(ControlMode.Velocity, -Constants.kBottomConveyorSpeed);
                }
                else{
                    mBottomCANTalonSRX.set(ControlMode.PercentOutput, -Constants.kBottomConveyorDutyCycle);
                }

                break;
            case kLaunching:
                if(SPEED_CONTROL_BOTTOM){
                    mBottomCANTalonSRX.set(ControlMode.Velocity, mLastIntakeDirection == ConveyorControlState.kFront ? Constants.kBottomLaunchSpeed : -Constants.kBottomLaunchSpeed);//Tells which direction to feed the intake while shooting

                }
                else{
                    mBottomCANTalonSRX.set(ControlMode.PercentOutput, mPeriodicIO.lastIntakeDirection == ConveyorControlState.kFront? Constants.kBottomConveyorDutyCycle:-Constants.kBottomConveyorDutyCycle);
                }
                if(SPEED_CONTROL_UPPER){
                    mTopCANTalonSRX.set(ControlMode.Velocity, Constants.kTopLaunchSpeed);
                }
                else{
                    mTopCANTalonSRX.set(ControlMode.PercentOutput, Constants.kTopConveyorDutyCycle);
                }
                if(mPeriodicIO.isReadyToLaunch){
                    if(SPEED_CONTROL_TRIGGER){
                        mTriggerCANTalonSRX.set(ControlMode.Velocity, Constants.kTriggerLaunchSpeed);
                    }
                    else{
                        mTriggerCANTalonSRX.set(ControlMode.PercentOutput, Constants.kTriggerDutyCycle);
                    }

                }

                break;
            case kPositionalControl:
                mTriggerCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mBottomCANTalonSRX.set(ControlMode.PercentOutput, 0);

                //turn the wheel at a speed until some condition is met to kick it out of that state
                mFrontSolenoid.set(DoubleSolenoid.Value.kForward);//check which intake it actually is going to be on
                mOuterIntakeSparkMAX.set(ControlType.kVelocity, Constants.kColorWheelRPM);
                if (mColorWheelReader.getMatchedColor() == mPeriodicIO.wantedColor)
                {
                    setControlState(ConveyorControlState.kIdle);
                }


                break;
            case kRotationalControl:
                mTriggerCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mBottomCANTalonSRX.set(ControlMode.PercentOutput, 0);

                //turn the wheel at a speed until some condition is met to kick it out of that state
                mFrontSolenoid.set(DoubleSolenoid.Value.kForward);//check which intake it actually is going to be on
                mOuterIntakeSparkMAX.set(ControlType.kVelocity, Constants.kColorWheelRPM);

                if(mColorWheelReader.getColorWheelRotationCount() > 3.0){
                    mOuterIntakeSparkMAX.set(ControlType.kVelocity, 0);
                    setControlState(ConveyorControlState.kIdle);
                }

                break;
        }
    }

    public void readPeriodicInputs(){
        mPeriodicIO.currentTimestamp = Timer.getFPGATimestamp();
        mPeriodicIO.triggerMotorVoltage = mTriggerCANTalonSRX.getMotorOutputVoltage();
        mPeriodicIO.topMotorVoltage = mTopCANTalonSRX.getMotorOutputVoltage();
        mPeriodicIO.bottomMotorVoltage = mBottomCANTalonSRX.getMotorOutputVoltage();

        mPeriodicIO.triggerMotorAmps = mTriggerCANTalonSRX.getStatorCurrent();
        mPeriodicIO.topMotorAmps = mTopCANTalonSRX.getStatorCurrent();
        mPeriodicIO.bottomMotorAmps = mBottomCANTalonSRX.getStatorCurrent();

        mPeriodicIO.triggerCurrentVelocity = mTriggerCANTalonSRX.getSelectedSensorVelocity();
        mPeriodicIO.topCurrentVelocity = mTopCANTalonSRX.getSelectedSensorVelocity();
        mPeriodicIO.bottomCurrentVelocity = mBottomCANTalonSRX.getSelectedSensorVelocity();

        mPeriodicIO.frontIsExtended = mFrontSolenoid.get() == DoubleSolenoid.Value.kForward;
        mPeriodicIO.backIsExtended = mBackSolenoid.get() == DoubleSolenoid.Value.kForward;

        mPeriodicIO.distance = mDistanceSensor.getRange();
        mPeriodicIO.wasEmpty = mPeriodicIO.isEmpty;
        mPeriodicIO.isEmpty = mPeriodicIO.distance > Constants.kEmptyDistance && mDistanceSensor.isRangeValid();
        mPeriodicIO.wasFull = mPeriodicIO.isFull;
        mPeriodicIO.isFull = mPeriodicIO.distance < Constants.kFullDistance && mDistanceSensor.isRangeValid();
        mPeriodicIO.timeEmptied = timeEmptied(mPeriodicIO.currentTimestamp);
        mPeriodicIO.timeFilled = timeFilled(mPeriodicIO.currentTimestamp);
        mPeriodicIO.isDoneShooting = doneShooting();
        mPeriodicIO.isTopDoneConveyoring = topDoneConveyoring();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    private double timeEmptied(double timestamp){
        if (!mPeriodicIO.wasEmpty && mPeriodicIO.isEmpty){
            return timestamp;
        } else {
            return mPeriodicIO.timeEmptied;
        }
    }

    private double timeFilled(double timestamp){
        if (!mPeriodicIO.wasFull && mPeriodicIO.isFull){
            return timestamp;
        } else {
            return mPeriodicIO.timeFilled;
        }
    }

    public boolean doneShooting(){
        return mPeriodicIO.isEmpty && mPeriodicIO.currentTimestamp - mPeriodicIO.timeEmptied > Constants.kTimeTillEmpty;
    }

    public boolean topDoneConveyoring(){
        return mPeriodicIO.isFull && mPeriodicIO.currentTimestamp - mPeriodicIO.timeFilled > Constants.kTimeTillInPosition;
    }

    public void zeroSensors(){
        mTriggerCANTalonSRX.setSelectedSensorPosition(0);
        mTopCANTalonSRX.setSelectedSensorPosition(0);
        mBottomCANTalonSRX.setSelectedSensorPosition(0);
    }

    private double ConvertNEORPMToControlPanelRPM(double NEORPM){
        return NEORPM * Constants.kNEORPMToControlPanelRPMConversionFactor;
    }

    private void setControlState(ConveyorControlState state) {
        mPeriodicIO.ConveyorState = state;
        mConveyorControlState = state;

    }

    public void stop(){
        setControlState(ConveyorControlState.kIdle);
    }

    public boolean checkSystem(){
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Conveyor Control State", mPeriodicIO.ConveyorState.toString());
        SmartDashboard.putBoolean("Front Intake Is Extended", mPeriodicIO.frontIsExtended);
        SmartDashboard.putBoolean("Back Intake Is Extended",mPeriodicIO.backIsExtended);
        SmartDashboard.putNumber("Distance Sensor Reading (Inches)", mPeriodicIO.distance);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

    }

    public synchronized void startLogging(){
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/CONVEYOR-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging(){
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }


    public synchronized void setWantRotationalControl(){
        if (mConveyorControlState != ConveyorControlState.kRotationalControl) {
            setControlState(ConveyorControlState.kRotationalControl);
            mColorWheelReader.startSensing();
        }

    }

    public synchronized  void setWantPositionalControl(){
        if(mConveyorControlState != ConveyorControlState.kPositionalControl)
        {
            setControlState(ConveyorControlState.kPositionalControl);
            mColorWheelReader.startSensing();
            if(mPeriodicIO.wantedColor == ColorWheelReader.MatchedColor.kUnknown)
            {
                mPeriodicIO.FMS_Color = DriverStation.getInstance().getGameSpecificMessage();

                if (mPeriodicIO.FMS_Color.charAt(0) == 'B') {
                    mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kBlue;
                } else if (mPeriodicIO.FMS_Color.charAt(0) == 'G') {
                    mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kGreen;
                } else if (mPeriodicIO.FMS_Color.charAt(0) == 'R') {
                    mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kRed;
                } else if (mPeriodicIO.FMS_Color.charAt(0) == 'Y') {
                    mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kYellow;
                } else {
                    mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kUnknown;
                }
            }
        }


    }

    public void setWantLaunch(boolean isReady){
        setControlState(ConveyorControlState.kLaunching);
        mPeriodicIO.isReadyToLaunch = isReady;
    }

    public void intakeFromFront (){
        mPeriodicIO.lastIntakeDirection = ConveyorControlState.kFront;
        setControlState(ConveyorControlState.kFront);

    }

    public void intakeFromBack (){
        mPeriodicIO.lastIntakeDirection = ConveyorControlState.kBack;
        setControlState(ConveyorControlState.kBack);

    }
    public boolean isDoneShooting (){
        return mPeriodicIO.isDoneShooting;
    }

}


