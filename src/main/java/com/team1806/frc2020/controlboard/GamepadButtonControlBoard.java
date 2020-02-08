package com.team1806.frc2020.controlboard;

import com.team1806.frc2020.Constants;
import com.team1806.lib.util.Deadband;
import com.team1806.lib.util.DelayedBoolean;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private final double kDeadband = 0.15;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kButtonGamepadPort);
        reset();
    }

    @Override
    public void reset() {

    }

    @Override
    public double getJogTurret() {
        double jog = mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    @Override
    public double getJogHood() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    @Override
    public double getManualLaunchControl() {
        double jog = mController.getAnalogTrigger(XboxController.Side.RIGHT);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    @Override
    public boolean getCloseShot() {
        return mController.getButton(XboxController.Button.A);
    }

    @Override
    public boolean getAutoLineShot() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public boolean getTrenchShot() {
        return mController.getButton(XboxController.Button.X);
    }

    @Override
    public boolean getLongShot() {
        return mController.getButton(XboxController.Button.Y);
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public boolean getWantsDeployClimber() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getWantsRotationalControl() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getWantsPositionalControl() {
        return mController.getDigitalTrigger(XboxController.Side.LEFT); }


}