package com.team1806.frc2020.auto.actions;

import com.team1806.frc2020.subsystems.Drive;

public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() {
        Drive.getInstance().forceDoneWithPath();
    }
}