package com.team1806.frc2020.auto.actions;

import com.team1806.frc2020.game.Shot;
import com.team1806.frc2020.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class PrepareShotAction implements Action {

    private Shot mShot;
    private double mTimeout;
    private double mStartTime;
    private Superstructure mSuperstructure;
    private boolean stopOnFinish;


    public PrepareShotAction(Shot shot, boolean stopOnFinish) {
        mShot = shot;
        mSuperstructure = Superstructure.getInstance();
        this.stopOnFinish = stopOnFinish;

    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.setPrepareShot(mShot);
    }

    @Override
    public void update() {


    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
        if(stopOnFinish) mSuperstructure.stop();
    }
}
