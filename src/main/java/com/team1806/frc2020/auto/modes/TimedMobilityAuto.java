package com.team1806.frc2020.auto.modes;

import com.team1806.frc2020.auto.AutoModeEndedException;
import com.team1806.frc2020.auto.actions.TimedMobilityAction;

public class TimedMobilityAuto extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new TimedMobilityAction(1.25));
    }
}
