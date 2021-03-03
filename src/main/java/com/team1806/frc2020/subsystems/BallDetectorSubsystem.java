package com.team1806.frc2020.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallDetectorSubsystem extends Subsystem {


    private Ultrasonic ultrasonic;

    private static BallDetectorSubsystem singleton = new BallDetectorSubsystem();

    public static BallDetectorSubsystem GetInstance(){
        return singleton;
    }

    public boolean isBallDetected(){

        return ultrasonic.getRangeInches() < 70 && ultrasonic.getRangeInches() >= 50;
    }

    private BallDetectorSubsystem(){
        ultrasonic = new Ultrasonic(0, 1);
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("IsBallDetected?", isBallDetected());
    }
}
