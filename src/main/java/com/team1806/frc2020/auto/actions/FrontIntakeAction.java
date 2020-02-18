package com.team1806.frc2020.auto.actions;

import com.team1806.frc2020.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class FrontIntakeAction {

    private double mTimeout;
    private double mStartTime;
    private Superstructure mSuperstructure;

    public FrontIntakeAction(double timeout){
        mTimeout = timeout;
        mSuperstructure = Superstructure.getInstance();
    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.frontIntake();

    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mTimeout;
    }

    @Override
    public void done() {mSuperstructure.stop();}
}
