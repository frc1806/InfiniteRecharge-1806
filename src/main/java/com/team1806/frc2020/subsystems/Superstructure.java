package com.team1806.frc2020.subsystems;

        import com.team1806.frc2020.Constants;
        import com.team1806.frc2020.Robot;
        import com.team1806.frc2020.RobotState;
        import com.team1806.frc2020.game.Shot;
        import com.team1806.frc2020.loops.ILooper;
        import com.team1806.frc2020.loops.Loop;
        import com.team1806.lib.geometry.Pose2d;
        import com.team1806.lib.geometry.Rotation2d;
        import com.team1806.lib.geometry.Twist2d;
        import com.team1806.lib.util.Units;
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
    private SuperstructureState mLauncherState;

    private enum SuperstructureState {
        kIdle,
        kLaunching,
        kVisionLaunching,
        kFrontIntake,
        kBackIntake
    }





    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {
        mLauncherState = SuperstructureState.kIdle;
        mFlywheel = Flywheel.GetInstance();
        mTurret = Turret.GetInstance();
        mHood = Hood.GetInstance();
        mConveyor = Conveyor.GetInstance();
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
                    switch(mLauncherState){
                        case kIdle:
                        default:
                            mFlywheel.stop();
                            mTurret.stop();
                            mHood.setWantedAngle(0);
                            break;
                        case kVisionLaunching:
                            mCurrentShot = getShotFromVision();
                            //intentional no break
                        case kLaunching:
                            mFlywheel.setSpeed(mCurrentShot.getFlywheelSpeed());
                            mTurret.setWantedAngle(mCurrentShot.getTurretAngle());
                            mHood.setWantedAngle(mCurrentShot.getHoodAngle());
                            if(mFlywheel.isReadyForLaunch() && mTurret.isOnTarget() && mHood.isOnTarget()){
                                mConveyor.setWantLaunch();
                            }
                            else{
                                mConveyor.stop();
                            }
                            break;
                        case kFrontIntake:
                            mFlywheel.stop();
                            mTurret.stop();
                            mHood.stop();

                            mConveyor.intakeFromFront();
                            break;
                        case kBackIntake:
                            mFlywheel.stop();
                            mTurret.stop();
                            mHood.stop();

                            mConveyor.intakeFromBack();
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
        setStopShooting();

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry() {}

    public synchronized void setWantShot(Shot wantedShot){
        mCurrentShot = wantedShot;
        if(mLauncherState != SuperstructureState.kLaunching)
        {
            mLauncherState = SuperstructureState.kLaunching;
        }


    }

    public synchronized  void setWantVisionShot(){
        if(mLauncherState != SuperstructureState.kVisionLaunching)
        {
            mLauncherState = SuperstructureState.kVisionLaunching;
        }
    }

    public synchronized  void setStopShooting(){
        if(mLauncherState != SuperstructureState.kIdle){
            mLauncherState = SuperstructureState.kIdle;
        }
    }

    private synchronized Shot getShotFromVision(){
        Pose2d goalPose = RobotState.getInstance().getVehicleToVisionTarget(Timer.getFPGATimestamp());
        Shot visionShot = new Shot(goalPose.getRotation().getDegrees(), getHoodAngleFromDistance(goalPose.getTranslation().norm()), getFlywheelSpeedFromDistance(goalPose.getTranslation().norm()));
        return visionShot;
    }

    public void frontIntake(){
        if(mLauncherState != SuperstructureState.kFrontIntake){
            mLauncherState = SuperstructureState.kFrontIntake;
        }
    }

    public void backIntake(){
        if(mLauncherState != SuperstructureState.kBackIntake){
            mLauncherState = SuperstructureState.kBackIntake;
        }
    }

    private double getHoodAngleFromDistance(double distance){
        return 0;
    }

    private double getFlywheelSpeedFromDistance(double distance){
        return 7806;
    }


}