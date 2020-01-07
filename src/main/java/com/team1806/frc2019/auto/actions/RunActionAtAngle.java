package com.team1806.frc2019.auto.actions;

import com.team1806.frc2019.RobotState;

public class RunActionAtAngle implements Action {
    private double triggerAngle;
    private double currentAngle;
    private double lastAngle;
    private Action action = null;
    private boolean hasRunAction = false;
    public RunActionAtAngle(double angle, Action action){
        this.triggerAngle = angle;
        this.action = action;
    }
    @Override
    public boolean isFinished() {
        return action.isFinished();
    }

    @Override
    public void update() {
        currentAngle = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        if((lastAngle <= triggerAngle && currentAngle > triggerAngle) || (currentAngle <= triggerAngle && lastAngle > triggerAngle) && !hasRunAction){
            action.start();
            hasRunAction = true;
        }
        if(hasRunAction){
            action.update();
        }
        lastAngle = currentAngle;
    }

    @Override
    public void done() {
        action.done();
    }

    @Override
    public void start() {
        lastAngle = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        currentAngle = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees();
    }
}
