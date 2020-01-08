package com.team1806.frc2020.auto.actions;

import com.team1806.frc2020.RobotState;

public class RunActionAtAngleRange implements Action {
    double minRange;
    double maxRange;
    private Action action = null;
    private boolean hasRunAction = false;
    double currentAngle;
    public RunActionAtAngleRange(double minAngle, double maxAngle, Action action){
        this.action = action;
        this.minRange = minAngle;
        this.maxRange = maxAngle;
    }
    @Override
    public boolean isFinished() {
        return action.isFinished();
    }

    @Override
    public void update() {
        currentAngle = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        if(currentAngle >= minRange && currentAngle <= maxRange && !hasRunAction){
            action.start();
            hasRunAction = true;
        }
        if(hasRunAction){
            action.update();
        }
    }

    @Override
    public void done() {
        action.done();
        System.out.println("Action  " + action.getClass().getName() + "Finished! Wow!");
    }

    @Override
    public void start() {

    }
}
