package com.team1806.frc2020.controlboard;

public interface IDebugControlBoard {
    void reset();
    public boolean getWantDashboardShot();
    public boolean getWantManualHood();
    public boolean getWantManualTurret();
    public boolean getWantTrigger();
}
