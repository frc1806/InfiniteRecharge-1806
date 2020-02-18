package com.team1806.frc2020.auto.actions;


import com.team1806.frc2020.game.Shot;
import com.team1806.frc2020.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class LaunchAction implements Action {

        private Shot mShot;
        private double mTimeout;
        private double mStartTime;
        private Superstructure mSuperstructure;


        public LaunchAction(Shot shot, double timeout){
            mShot = shot;
            mTimeout = timeout;
            mSuperstructure = Superstructure.getInstance();

        }


        @Override
        public void start() {
            mStartTime = Timer.getFPGATimestamp();
            mSuperstructure.setWantShot(mShot);
        }

        @Override
        public void update() {


        }

        @Override
        public boolean isFinished() {
           return mSuperstructure.isDoneShooting() || Timer.getFPGATimestamp() - mStartTime > mTimeout;
        }

        @Override
        public void done() {
            mSuperstructure.stop();
        }
    }