package com.team1806.frc2020.auto.modes;

import com.team1806.frc2020.auto.AutoModeEndedException;
import com.team1806.frc2020.auto.actions.LaunchAction;
import com.team1806.frc2020.auto.actions.TimedMobilityAction;
import com.team1806.frc2020.game.Shot;

public class StraightOnLaunchAndMobilityAuto extends AutoModeBase {


    protected void routine() throws AutoModeEndedException {
        runAction(new LaunchAction(Shot.STRAIGHT_ON_AUTOLINE, 10));
        runAction(new TimedMobilityAction(1.25));
    }

}
