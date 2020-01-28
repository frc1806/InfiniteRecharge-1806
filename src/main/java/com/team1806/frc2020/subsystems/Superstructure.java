package com.team1806.frc2020.subsystems;

        import com.team1806.frc2020.Constants;
        import com.team1806.frc2020.RobotState;
        import com.team1806.frc2020.loops.ILooper;
        import com.team1806.frc2020.loops.Loop;
        import com.team1806.lib.geometry.Pose2d;
        import com.team1806.lib.geometry.Rotation2d;
        import com.team1806.lib.geometry.Twist2d;
        import com.team1806.lib.util.Units;
        import com.team1806.lib.util.Util;
        import com.team1806.lib.vision.AimingParameters;

        import java.util.Optional;

/**
 * The superstructure subsystem is the overarching class containing all components of the superstructure: the
 * turret, elevator, arm, and wrist. The superstructure subsystem also uses info from the vision system.
 * <p>
 * Instead of interacting individually with subsystems like the elevator and arm, the {@link Robot} class sends commands
 * to the superstructure, which individually decides how to move each subsystem to get there.
 * <p>
 * The Superstructure class also adjusts the overall goal based on the turret and elevator control modes.
 *
 * @see com.team1806.frc2020.statemachines.SuperstructureCommands
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


    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {
        mFlywheel = Flywheel.GetInstance();
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
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry() {}

}