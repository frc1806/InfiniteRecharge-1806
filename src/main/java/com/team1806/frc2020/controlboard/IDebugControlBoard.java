package com.team1806.frc2020.controlboard;

public interface IDebugControlBoard {
    void reset();

    boolean getWantDashboardShot();

    boolean getWantManualHood();

    boolean getWantManualTurret();

    boolean getWantTrigger();
}
