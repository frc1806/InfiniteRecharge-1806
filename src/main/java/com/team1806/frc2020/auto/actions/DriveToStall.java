package com.team1806.frc2020.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import com.team1806.frc2020.auto.actions.Action;
import com.team1806.frc2020.subsystems.Drive;

public class DriveToStall implements Action {
    Drive mDrive = Drive.getInstance();
    @Override
    public boolean isFinished() {
        return mDrive.driveToStall(false, true);
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mDrive.driveToStall(true, false);
    }
}
