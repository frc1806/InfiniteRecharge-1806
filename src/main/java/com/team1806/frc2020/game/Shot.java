package com.team1806.frc2020.game;

public class Shot {

    public static Shot CLOSE_SHOT = new Shot(0, 0, 5000);
    public static Shot STRAIGHT_ON_AUTOLINE = new Shot(0, 10, 8000);
    public static Shot FROM_TRENCH =  new Shot(-5, 10, 8000);
    public static Shot STRAIGHT_ON_FROM_MAX_DIST = new Shot(0, 30, 10000);
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
