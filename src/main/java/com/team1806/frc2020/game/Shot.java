package com.team1806.frc2020.game;

public class Shot {

    public static Shot CLOSE_SHOT = new Shot(0, 17, 2500);
    public static Shot STRAIGHT_ON_AUTOLINE = new Shot(0, 47.5, 3500);
    public static Shot FROM_TRENCH = new Shot(0, 53, 4500);
    public static Shot STRAIGHT_ON_FROM_MAX_DIST = new Shot(0, 52, 5500);
    private double mTurretAngle;
    private double mHoodAngle;
    private double mFlywheelSpeed;

    public Shot(double mTurretAngle, double mHoodAngle, double mFlywheelSpeed) {
        this.mTurretAngle = mTurretAngle;
        this.mHoodAngle = mHoodAngle;
        this.mFlywheelSpeed = mFlywheelSpeed;
    }

    public double getTurretAngle() {
        return mTurretAngle;
    }

    public double getHoodAngle() {
        return mHoodAngle;
    }

    public double getFlywheelSpeed() {
        return mFlywheelSpeed;
    }
}
