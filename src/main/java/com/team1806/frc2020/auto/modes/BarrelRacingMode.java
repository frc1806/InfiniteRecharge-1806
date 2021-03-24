package com.team1806.frc2020.auto.modes;

import com.team1806.frc2020.auto.AutoModeEndedException;
import com.team1806.frc2020.auto.actions.DrivePathAction;
import com.team1806.frc2020.paths.PathBarrelRace;

public class BarrelRacingMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DrivePathAction(new PathBarrelRace()));
    }
}
