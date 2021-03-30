package com.team1806.frc2020.controlboard;

public interface IDriveControlBoard {
    //Cheezy Drive
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();
    
    //Skid Steer
    double getLeftThrottle();

    double getRightThrottle();

    boolean getShoot();

    boolean getWantsLowGear();

    boolean getWantsFrontIntake();

    boolean getWantsRearIntake();

    void setRumble(boolean on);

    boolean getWantsPark();

    boolean getWantsAutoSteer();

    boolean getWantVisionShot();

    boolean getWantsHighGear();

    boolean getCloseShot();

    boolean getWantSingleShot();
}