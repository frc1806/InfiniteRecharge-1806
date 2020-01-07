package com.team1806.frc2019.auto.actions;

import com.team1806.frc2019.auto.actions.Action;
import com.team1806.frc2019.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team1806.frc2019.Constants;

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
    public boolean isFinished() {return action.isFinished();}



    @Override
    public void update() {
    currentAcceleration = mDriveTrainSubsystem.getWorldLinearAccelZ();
    if (currentAcceleration - lastAcceleration >= Constants.kCollisionJerkThreshold && !hasActionRun){
        action.start();
        hasActionRun = true;
    }
    if (hasActionRun){
        action.update();
    }
    lastAcceleration = currentAcceleration;
    }


    @Override
    public void done() { action.done();}

    @Override
    public void start() {

    }

}