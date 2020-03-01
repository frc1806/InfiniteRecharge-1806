package com.team1806.frc2020.auto.actions;

import com.team1806.frc2020.Constants;
import com.team1806.frc2020.subsystems.Drive;

public class RunActionAfterCollision implements Action {

    private Drive mDriveTrainSubsystem;
    private Action action = null;
    private boolean hasActionRun;
    private float lastAcceleration = 0;
    private float currentAcceleration = 0;


    public RunActionAfterCollision(Action actionToRun) {
        mDriveTrainSubsystem = Drive.getInstance();
        action = actionToRun;
        hasActionRun = false;

    }

    @Override
    public boolean isFinished() {
        return action.isFinished();
    }


    @Override
    public void update() {
        currentAcceleration = mDriveTrainSubsystem.getWorldLinearAccelZ();
        if (currentAcceleration - lastAcceleration >= Constants.kCollisionJerkThreshold && !hasActionRun) {
            action.start();
            hasActionRun = true;
        }
        if (hasActionRun) {
            action.update();
        }
        lastAcceleration = currentAcceleration;
    }


    @Override
    public void done() {
        action.done();
    }

    @Override
    public void start() {

    }

}