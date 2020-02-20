package com.team1806.frc2020.controlboard;

import com.team1806.frc2020.Constants;
import com.team1806.lib.util.LatchedBoolean;

public class GamepadDebugControlBoard implements IDebugControlBoard {
    private final double kDeadband = 0.15;
    private final XboxController mController;
    private LatchedBoolean wantManualHood = new LatchedBoolean();
    private LatchedBoolean wantManualTurret = new LatchedBoolean();
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

    @Override
    public boolean getWantManualHood() { ;
        return wantManualHood.update(mController.getButton(XboxController.Button.X));
    }

    @Override
    public boolean getWantManualTurret() {
        return wantManualTurret.update(mController.getButton(XboxController.Button.B));
    }
}
