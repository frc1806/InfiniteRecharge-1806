package com.team1806.frc2020.subsystems;

import com.team1806.frc2020.Robot;
import com.team1806.frc2020.RobotState;
import com.team1806.frc2020.controlboard.ControlBoard;
import com.team1806.frc2020.game.Shot;
import com.team1806.frc2020.loops.ILooper;
import com.team1806.frc2020.loops.Loop;
import com.team1806.lib.geometry.Pose2d;
import com.team1806.lib.geometry.Rotation2d;
import com.team1806.lib.vision.AimingParameters;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

/**
 * The superstructure subsystem is the overarching class containing all components of the superstructure: the
 * turret, flywheel, hood, and conveyor.
 * <p>
 * Instead of interacting individually with subsystems like the flywheel, turret, and hood, the {@link Robot} class sends commands
 * to the superstructure, which individually decides how to move each subsystem to get there.
 * <p>
 */
public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    private final RobotState mRobotState = RobotState.getInstance();


    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private double mCorrectedRangeToTarget = 0.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;
    private Flywheel mFlywheel;
    private Conveyor mConveyor;
    private Hood mHood;
    private Turret mTurret;
    private Shot mCurrentShot;
    private SuperstructureState mSuperstructureState;
    private boolean mWantInnerGoal = false;
    private boolean mWantSingleShot = false;

    private Superstructure() {
        mSuperstructureState = SuperstructureState.kIdle;
        mFlywheel = Flywheel.GetInstance();
        mTurret = Turret.GetInstance();
        mHood = Hood.GetInstance();
        mConveyor = Conveyor.GetInstance();
    }

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    public boolean getWantSingleShot() {
        return mWantSingleShot;
    }

    public void setWantSingleShot(boolean mWantSingleShot) {
        this.mWantSingleShot = mWantSingleShot;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    switch (mSuperstructureState) {
                        case kIdle:
                        default:
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                if (!mHood.isRetracting()) {
                                    mHood.setWantedAngle(0);
                                }
                            }
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.stop();
                            }
                            mFlywheel.stop();
                            mConveyor.stop();
                            break;
                        case kVisionLaunching:
                            mCurrentShot = getShotFromVision(mWantInnerGoal);
                            //intentional no break
                        case kLaunching:
                            mFlywheel.setSpeed(mCurrentShot.getFlywheelSpeed());
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.setWantedAngle(mCurrentShot.getTurretAngle());
                            }
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                mHood.setWantedAngle(mCurrentShot.getHoodAngle());
                            }
                            mConveyor.setWantLaunch(mFlywheel.isReadyForLaunch()); //&& mTurret.isOnTarget() && mHood.isOnTarget());
                            mConveyor.setWantSingleShot(mWantSingleShot);
                            break;
                        case kPreparingShot:
                            mFlywheel.setSpeed(mCurrentShot.getFlywheelSpeed());
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.setWantedAngle(mCurrentShot.getTurretAngle());
                            }
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                mHood.setWantedAngle(mCurrentShot.getHoodAngle());
                            }
                            break;
                        case kFrontIntake:
                            mFlywheel.stop();
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.stop();
                            }
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                if (!mHood.isRetracting()) {
                                    mHood.setWantedAngle(0);
                                }
                            }

                            mConveyor.intakeFromFront();
                            break;
                        case kBackIntake:
                            mFlywheel.stop();
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.stop();
                            }
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                if (!mHood.isRetracting()) {
                                    mHood.setWantedAngle(0);
                                }
                            }

                            mConveyor.intakeFromBack();
                            break;
                        case kUnjamming:
                            mConveyor.setWantUnjam();
                            mFlywheel.stop();
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.stop();
                            }
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                if (!mHood.isRetracting()) {
                                    mHood.setWantedAngle(0);
                                }
                            }
                            break;
                        case kIntakeSweep:
                            mConveyor.wantSweep();
                            mFlywheel.stop();
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.stop();
                            }
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                if (!mHood.isRetracting()) {
                                    mHood.setWantedAngle(0);
                                }
                            }
                            break;
                        case kAgitate:
                            mConveyor.setWantAgitate();
                            mFlywheel.stop();
                            if (!ControlBoard.GetInstance().getWantManualTurret()) {
                                mTurret.stop();
                            }
                            if (!ControlBoard.GetInstance().getWantManualHood()) {
                                if (!mHood.isRetracting()) {
                                    mHood.setWantedAngle(0);
                                }
                            }
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    public synchronized Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
    }

    public synchronized int getTrackId() {
        /*if (getCurrentlyAiming()) {
            return mTrackId;
        }*/
        return -1;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint, boolean enforce_min_distance, double min_distance) {
        mEnforceAutoAimMinDistance = enforce_min_distance;
        mAutoAimMinDistance = min_distance;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint) {
        setWantAutoAim(field_to_turret_hint, false, 500);
    }

    @Override
    public void stop() {
        mSuperstructureState = SuperstructureState.kIdle;

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry() {
    }

    public synchronized void setWantShot(Shot wantedShot) {
        mCurrentShot = wantedShot;
        if (mSuperstructureState != SuperstructureState.kLaunching) {
            mSuperstructureState = SuperstructureState.kLaunching;
        }
    }

    public synchronized void setPrepareShot(Shot wantedShot) {
        mCurrentShot = wantedShot;
        if (mSuperstructureState != SuperstructureState.kPreparingShot) {
            mSuperstructureState = SuperstructureState.kPreparingShot;
        }
    }

    public synchronized void setWantVisionShot(boolean innerGoal) {
        mWantInnerGoal = innerGoal;
        if (mSuperstructureState != SuperstructureState.kVisionLaunching) {
            mSuperstructureState = SuperstructureState.kVisionLaunching;
        }
    }

    public synchronized void setStopShooting() {
        if (mSuperstructureState != SuperstructureState.kIdle) {
            mSuperstructureState = SuperstructureState.kIdle;
        }
    }

    private synchronized Shot getShotFromVision(boolean innerGoal) {
        Pose2d goalPose = RobotState.getInstance().getVehicleToVisionTarget(Timer.getFPGATimestamp(), innerGoal);
        Shot visionShot = new Shot(goalPose.getRotation().getDegrees(), getHoodAngleFromDistance(goalPose.getTranslation().norm()), getFlywheelSpeedFromDistance(goalPose.getTranslation().norm()));
        return visionShot;
    }

    public void frontIntake() {
        if (mSuperstructureState != SuperstructureState.kFrontIntake) {
            mSuperstructureState = SuperstructureState.kFrontIntake;
        }
    }

    public void backIntake() {
        if (mSuperstructureState != SuperstructureState.kBackIntake) {
            mSuperstructureState = SuperstructureState.kBackIntake;
        }
    }

    public void unjam() {
        if (mSuperstructureState != SuperstructureState.kUnjamming) {
            mSuperstructureState = SuperstructureState.kUnjamming;
        }
    }

    public void setWantSweep() {
        if (mSuperstructureState != SuperstructureState.kIntakeSweep) {
            mSuperstructureState = SuperstructureState.kIntakeSweep;
        }
    }
    public void setWantAgitate(){
        if(mSuperstructureState != SuperstructureState.kAgitate){
            mSuperstructureState = SuperstructureState.kAgitate;
        }
    }

    public boolean isDoneShooting() {
        return mConveyor.isDoneShooting();
    }

    private double getHoodAngleFromDistance(double distance) {
        return 0;
    }

    private double getFlywheelSpeedFromDistance(double distance) {
        return 3500;
    }

    private enum SuperstructureState {
        kIdle,
        kLaunching,
        kPreparingShot,
        kVisionLaunching,
        kFrontIntake,
        kBackIntake,
        kUnjamming,
        kIntakeSweep,
        kAgitate
    }


}