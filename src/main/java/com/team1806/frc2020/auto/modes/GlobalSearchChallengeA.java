package com.team1806.frc2020.auto.modes;

import java.util.Arrays;

import com.team1806.frc2020.auto.AutoModeEndedException;
import com.team1806.frc2020.auto.actions.BackIntakeAction;
import com.team1806.frc2020.auto.actions.DrivePathAction;
import com.team1806.frc2020.auto.actions.ParallelAction;
import com.team1806.frc2020.paths.RedPathA;
import com.team1806.frc2020.subsystems.BallDetectorSubsystem;

public class GlobalSearchChallengeA extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // TODO Auto-generated method stub
        if(BallDetectorSubsystem.GetInstance().isBallDetected()){
            //red
            runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new RedPathA()), new BackIntakeAction(0))));
        }
        //else{ runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new bluePathA()), new BackIntakeAction(15))));
            //blue
        }
    }
