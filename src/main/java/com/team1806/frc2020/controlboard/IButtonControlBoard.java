package com.team1806.frc2020.controlboard;

import com.team1806.lib.geometry.Rotation2d;

public interface IButtonControlBoard {
    void reset();

    double getJogTurret();
    double getJogHood();
    double getManualLaunchControl();

    //Preset Launcher
    boolean getCloseShot();

    boolean getAutoLineShot();

    boolean getTrenchShot();

    boolean getLongShot();

    void setRumble(boolean on);

    // Climbing
    boolean getWantsDeployClimber();

    //Control Wheel Manip
    boolean getWantsRotationalControl();

    boolean getWantsPositionalControl();

    //Conveyor
    boolean getWantsUnjam();

}