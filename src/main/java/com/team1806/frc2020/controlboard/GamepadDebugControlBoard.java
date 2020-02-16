package com.team1806.frc2020.controlboard;

import com.team1806.frc2020.Constants;

public class GamepadDebugControlBoard implements IDebugControlBoard {
    private final double kDeadband = 0.15;
    private final XboxController mController;

    private static GamepadDebugControlBoard mInstance = null;

    public static GamepadDebugControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDebugControlBoard();
        }

        return mInstance;
    }

    private GamepadDebugControlBoard() {
        mController = new XboxController(Constants.kDebugGamepadPort);
        reset();
    }

    @Override
    public void reset() {

    }

    @Override
    public boolean getWantDashboardShot() {
        return mController.getButton(XboxController.Button.A);
    }
}
