package com.team1806.frc2020.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getShoot();

    boolean getWantsLowGear();

    boolean getWantsFrontIntake();

    boolean getWantsRearIntake();

    void setRumble(boolean on);

    boolean getWantsPark();

    boolean getWantsAutoSteer();
}