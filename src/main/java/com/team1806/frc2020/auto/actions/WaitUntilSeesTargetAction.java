package com.team1806.frc2020.auto.actions;

import com.team1806.frc2020.Constants;
import com.team1806.frc2020.RobotState;

public class WaitUntilSeesTargetAction implements Action {
    @Override
    public void start() {
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getAimingParameters(-1, Constants.kMaxGoalTrackAge).isPresent();
    }

    @Override
    public void done() {
    }
}