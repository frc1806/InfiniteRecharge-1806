package com.team1806.frc2020.game;

public class Shot {

    public static Shot CLOSE_SHOT = new Shot(0, 17, 2500);
    public static Shot STRAIGHT_ON_AUTOLINE = new Shot(0, 47.5, 3500);
    public static Shot FROM_TRENCH = new Shot(0, 53, 4500);
    public static Shot STRAIGHT_ON_FROM_MAX_DIST = new Shot(0, 52, 5500);
    public static Shot GREEN_ZONE = new Shot(0, 20, 2000);
    public static Shot YELLOW_ZONE = new Shot(0,43,4000);
    public static Shot BLUE_ZONE = new Shot(0,25,3000);
    public static Shot RED_ZONE = new Shot(0,35,2450);
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
