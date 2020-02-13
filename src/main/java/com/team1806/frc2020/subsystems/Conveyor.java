package com.team1806.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.team1806.frc2020.Constants;
import com.team1806.lib.drivers.LazySparkMax;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

public class Conveyor extends Subsystem {

    enum ConveyorControlState {

        kIdle, kFront, kBack, kLaunching, kPositionalControl, kRotationalControl
    }


    private class PeriodicIO {
        public double timestamp;

        public double triggerMotorVoltage;
        public double topMotorVoltage;
        public double bottomMotorVoltage;

        public double triggerMotorAmps;
        public double topMotorAmps;
        public double bottomMotorAmps;

        public double triggerCurrentVelocity;
        public double topCurrentVelocity;
        public double bottomCurrentVelocity;

        public boolean frontIsExtended;
        public boolean backIsExtended;
        public ColorWheelReader.MatchedColor wantedColor;

        public ConveyorControlState ConveyorState;

        public ConveyorControlState lastIntakeDirection;

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



    private Conveyor(){
        mPeriodicIO = new PeriodicIO();
        mPeriodicIO.wantedColor = ColorWheelReader.MatchedColor.kUnknown;

        mTriggerCANTalonSRX = new TalonSRX(Constants.kTriggerConveyorMotorId);
        mTopCANTalonSRX = new TalonSRX(Constants.kTopConveyorMotorId);
        mBottomCANTalonSRX = new TalonSRX(Constants.kBottomConveyorMotorId);
        mOuterIntakeSparkMAX = new LazySparkMax(Constants.kOuterIntake);

        mFrontSolenoid = new DoubleSolenoid(Constants.kFrontIntakeFowardChannel, Constants.kFrontIntakeReverseChannel);
        mBackSolenoid = new DoubleSolenoid(Constants.kBackIntakeFowardChannel, Constants.kBackIntakeReverseChannel);

        mConveyorControlState = ConveyorControlState.kIdle;
        mLastIntakeDirection = ConveyorControlState.kFront;

        mTriggerCANTalonSRX.setNeutralMode(NeutralMode.Brake);
        mTopCANTalonSRX.setNeutralMode(NeutralMode.Brake);
        mBottomCANTalonSRX.setNeutralMode(NeutralMode.Brake);

        mOuterIntakeSparkMAX.setIdleMode(CANSparkMax.IdleMode.kBrake);




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
            mTopCANTalonSRX.config_kI(0, Constants.kTopConveyoVelocityControlKi);
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
        switch (mPeriodicIO.ConveyorState){
            default:
            case kIdle:
                mTriggerCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mBottomCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mFrontSolenoid.set(DoubleSolenoid.Value.kReverse);
                mBackSolenoid.set(DoubleSolenoid.Value.kReverse);

                break;
            case kFront:
                if (!mPeriodicIO.frontIsExtended){
                    mFrontSolenoid.set(DoubleSolenoid.Value.kForward);
                }


                break;
            case kBack:
                if (!mPeriodicIO.backIsExtended){
                    mBackSolenoid.set(DoubleSolenoid.Value.kForward);
                }

                break;
            case kLaunching:
                mBottomCANTalonSRX.set(ControlMode.Velocity, mLastIntakeDirection == ConveyorControlState.kFront? Constants.kTriggerLaunchSpeed:-Constants.kTriggerLaunchSpeed);
                mTopCANTalonSRX.set(ControlMode.Velocity, Constants.kBottomConveyorSpeed);
                mTriggerCANTalonSRX.set(ControlMode.Velocity, Constants.kTriggerLaunchSpeed);

                break;
            case kPositionalControl:
                mTriggerCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mBottomCANTalonSRX.set(ControlMode.PercentOutput, 0);

                //turn the wheel at a speed until some condition is met to kick it out of that state
                mFrontSolenoid.set(DoubleSolenoid.Value.kForward);//check which intake it actually is going to be on
                mOuterIntakeSparkMAX.set(ControlType.kVelocity, Constants.kColorWheelRPM);

                setWantPositionalControl();

                if(mPeriodicIO.wantedColor.length() > 0)
                {
                    switch (mPeriodicIO.wantedColor.charAt(0))
                    {
                        case 'B' :
                            //Blue case code
                            break;
                        case 'G' :
                            //Green case code
                            break;
                        case 'R' :
                            //Red case code
                            break;
                        case 'Y' :
                            //Yellow case code
                            break;
                    }
                } else {
                    setControlState(ConveyorControlState.kIdle);
                    System.out.println("*********NO GAME DATA*********Recieved String: \"" + mPeriodicIO.wantedColor);
                }


                break;
            case kRotationalControl:
                mTriggerCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mTopCANTalonSRX.set(ControlMode.PercentOutput, 0);
                mBottomCANTalonSRX.set(ControlMode.PercentOutput, 0);

                //turn the wheel at a speed until some condition is met to kick it out of that state
                mFrontSolenoid.set(DoubleSolenoid.Value.kForward);//check which intake it actually is going to be on
                mOuterIntakeSparkMAX.set(ControlType.kVelocity, Constants.kColorWheelRPM);

                if(mColorWheelReader.getColorWheelRotationCount() > 3){
                    mOuterIntakeSparkMAX.set(ControlType.kVelocity, 0);
                    setControlState(ConveyorControlState.kIdle);
                }

                break;
        }
    }

    public void readPeriodicInputs(){
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

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }



    public void zeroSensors(){
        mTriggerCANTalonSRX.setSelectedSensorPosition(0);
        mTopCANTalonSRX.setSelectedSensorPosition(0);
        mBottomCANTalonSRX.setSelectedSensorPosition(0);
    }

    private double ConvertNEORPMToControlPanelRPM(double NEORPM){
        return NEORPM * Constants.kNEORPMToControlPanelRPMConversionFactor;
    }

    public void setControlState(ConveyorControlState state) {
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
        setControlState(ConveyorControlState.kRotationalControl);
        mColorWheelReader.startSensing();
    }

    public synchronized  void setWantPositionalControl(){
        setControlState(ConveyorControlState.kPositionalControl);
        mColorWheelReader.startSensing();

        mPeriodicIO.wantedColor = DriverStation.getInstance().getGameSpecificMessage();
    }

}


