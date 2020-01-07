package com.team1806.frc2019.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getShoot();

    boolean getWantsLowGear();

    boolean getThrust();
}