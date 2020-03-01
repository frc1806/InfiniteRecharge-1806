package com.team1806.frc2020.controlboard;

import com.team1806.frc2020.Constants;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;
    private final XboxController mController;

    private GamepadDriveControlBoard() {
        mController = new XboxController(Constants.kDriveGamepadPort);
    }

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    @Override
    public boolean getWantsLowGear() {
        return mController.getDPad() == 180;
    }


    @Override
    public boolean getWantsFrontIntake() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getWantsRearIntake() {
        return mController.getButton(XboxController.Button.LB);
    }


    @Override
    public boolean getQuickTurn() {
        return mController.getDigitalTrigger(XboxController.Side.LEFT);
    }

    @Override
    public boolean getShoot() {
        return mController.getDigitalTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getWantsPark() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public boolean getWantsAutoSteer() {
        return mController.getButton(XboxController.Button.X);
    }

    @Override
    public boolean getWantVisionShot() {
        return mController.getButton(XboxController.Button.START);
    }

    @Override
    public boolean getWantsHighGear() {
        return mController.getDPad() == 0;
    }

    @Override
    public boolean getCloseShot() {
        return mController.getDPad() == 270;
    }
}