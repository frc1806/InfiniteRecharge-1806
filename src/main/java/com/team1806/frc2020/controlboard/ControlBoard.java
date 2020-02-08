package com.team1806.frc2020.controlboard;

import com.team1806.frc2020.Constants;

public class ControlBoard implements IControlBoard{

    private static final ControlBoard CONTROL_BOARD = new ControlBoard();
    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    public static ControlBoard GetInstance(){
        return CONTROL_BOARD;
    }

    private ControlBoard() {
        mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public void reset() {

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
    public void setRumble(boolean on) {
        mDriveControlBoard.setRumble(on);
        mButtonControlBoard.setRumble(on);
    }
}
