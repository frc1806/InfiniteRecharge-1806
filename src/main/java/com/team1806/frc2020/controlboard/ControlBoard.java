package com.team1806.frc2020.controlboard;

import com.team1806.frc2020.Constants;

public class ControlBoard implements IControlBoard{

    private static final ControlBoard CONTROL_BOARD = new ControlBoard();
    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;
    private final IDebugControlBoard mDebugControlBoard;

    public static ControlBoard GetInstance(){
        return CONTROL_BOARD;
    }

    private ControlBoard() {
        mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
        mDebugControlBoard = GamepadDebugControlBoard.getInstance();
    }

    @Override
    public void reset() {

    }

    @Override
    public boolean getWantDashboardShot() {
        return mDebugControlBoard.getWantDashboardShot();
    }

    @Override
    public boolean getWantManualHood() {
        return mDebugControlBoard.getWantManualHood();
    }

    @Override
    public boolean getWantManualTurret() {
        return mDebugControlBoard.getWantManualTurret();
    }

    @Override
    public double getJogTurret() {
        return mButtonControlBoard.getJogTurret();
    }

    @Override
    public double getJogHood() {
        return mButtonControlBoard.getJogHood();
    }

    @Override
    public double getManualLaunchControl() {
        return mButtonControlBoard.getManualLaunchControl();
    }

    @Override
    public boolean getCloseShot() {
        return mButtonControlBoard.getCloseShot();
    }

    @Override
    public boolean getAutoLineShot() {
        return mButtonControlBoard.getAutoLineShot();
    }

    @Override
    public boolean getTrenchShot() {
        return mButtonControlBoard.getTrenchShot();
    }

    @Override
    public boolean getLongShot() {
        return mButtonControlBoard.getLongShot();
    }

    @Override
    public boolean getWantsDeployClimber() {
        return mButtonControlBoard.getWantsDeployClimber();
    }

    @Override
    public boolean getWantsRotationalControl() {
        return mButtonControlBoard.getWantsRotationalControl();
    }

    @Override
    public boolean getWantsPositionalControl() {
        return mButtonControlBoard.getWantsPositionalControl();
    }

    @Override
    public boolean getWantsUnjam() { return mButtonControlBoard.getWantsUnjam(); }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getShoot() {
        return mDriveControlBoard.getShoot();
    }

    @Override
    public boolean getWantsLowGear() {
        return mDriveControlBoard.getWantsLowGear();
    }

    @Override
    public boolean getWantsFrontIntake() {
        return mDriveControlBoard.getWantsFrontIntake();
    }

    @Override
    public boolean getWantsRearIntake() {
        return mDriveControlBoard.getWantsRearIntake();
    }

    @Override
    public boolean getWantsPark(){return mDriveControlBoard.getWantsPark();}

    @Override
    public boolean getWantsAutoSteer() {
        return mDriveControlBoard.getWantsAutoSteer();
    }

    @Override
    public boolean getWantVisionShot(){
        return mDriveControlBoard.getWantVisionShot();
    }

    @Override
    public boolean getWantsFlashlight(){ return mButtonControlBoard.getWantsFlashlight(); }

    @Override
    public boolean getWantTrigger(){return mDebugControlBoard.getWantTrigger(); }

    @Override
    public boolean getWantIntakeSweep(){return mButtonControlBoard.getWantIntakeSweep();}

    @Override
    public void setRumble(boolean on) {
        mDriveControlBoard.setRumble(on);
        mButtonControlBoard.setRumble(on);
    }
}
