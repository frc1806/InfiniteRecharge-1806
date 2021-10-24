package com.team1806.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team1806.frc2020.Constants;
import com.team1806.frc2020.loops.ILooper;
import com.team1806.frc2020.loops.Loop;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber extends Subsystem {

    private Climber THE_CLIMBER = new Climber();

    private ClimberState climberState;

    private CANSparkMax motor;
    private DoubleSolenoid solenoid;


    private double wantedMotorDemand;
    private double lastWantedUnfold;
    private double lastTimestamp;

    public Climber GetInstance(){
        return THE_CLIMBER;
    }

    private Climber()
    {
        motor = new CANSparkMax(25, MotorType.kBrushless);
        solenoid = new DoubleSolenoid(7, 8);
        climberState = ClimberState.kFolded;
        wantedMotorDemand = 0;
        lastWantedUnfold = 0;
        lastTimestamp = 0;
    }

    @Override
    public void stop() {
        motor.set(0);

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

    @Override
    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
    }
    
    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
        
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                lastTimestamp = timestamp;

                if(solenoid.get() == Value.kForward && timestamp > lastWantedUnfold + Constants.kUnfoldTime)
                {
                    climberState = ClimberState.kUnfolded;
                }

                switch(climberState){
                    default:
                    case kDisabled:
                        motor.set(0);
                        break;
                    case kFolded:
                        motor.set(0);
                        break;
                    case kUnfolded:
                        motor.set(wantedMotorDemand);

                }

            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public void wantUnfold(){
        lastWantedUnfold = lastTimestamp;
        solenoid.set(Value.kForward);
    }

    public void wantFold(){
        climberState = ClimberState.kFolded;
        solenoid.set(Value.kReverse);
    }

    public void setWantedDemand(double power){
        wantedMotorDemand= power;
    }
    
    @Override
    public void zeroSensors() {
    }

    public enum ClimberState{
        kFolded,
        kUnfolded,
        kDisabled
    }
    
}