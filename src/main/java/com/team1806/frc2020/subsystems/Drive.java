package com.team1806.frc2020.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import com.team1806.frc2020.Constants;
import com.team1806.frc2020.Kinematics;
import com.team1806.frc2020.RobotState;
import com.team1806.frc2020.loops.ILooper;
import com.team1806.frc2020.loops.Loop;
import com.team1806.lib.control.Lookahead;
import com.team1806.lib.control.Path;
import com.team1806.lib.control.PathFollower;
import com.team1806.lib.drivers.LazySparkMax;
import com.team1806.lib.drivers.MotorChecker;
import com.team1806.lib.drivers.SparkMaxChecker;
import com.team1806.lib.drivers.SparkMaxFactory;
import com.team1806.lib.drivers.NavX;
import com.team1806.lib.geometry.*;
import com.team1806.lib.util.*;
import com.team1806.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Optional;

public class Drive extends Subsystem {
    private static Drive mInstance;

    //PID Slots
    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    // hardware
    private final LazySparkMax mLeftLeader, mRightLeader;
    ArrayList<LazySparkMax> mLeftFollowers = new ArrayList<>();
    ArrayList<LazySparkMax> mRightFollowers = new ArrayList<>();

    private final CANEncoder mLeftEncoder, mRightEncoder;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    private final Solenoid mShifter;
    // control states
    private DriveControlState mDriveControlState;
    private DriveCurrentLimitState mDriveCurrentLimitState;
    //private PigeonIMU mPigeon;
    private NavX mNavx;
    // hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private double mLastDriveCurrentSwitchTime = -1;

    //timing
    private double currentTimeStamp;
	private double lastTimeStamp;

    //drive to stall variables
	boolean wasPushing = false;
	boolean startingPush = false;
	boolean finishingPush = false;
	boolean isTimedOut = false;
	double pushTimeStamp = 0;

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
        sparkMax.setInverted(!left);
        sparkMax.enableVoltageCompensation(12.0);
        sparkMax.setClosedLoopRampRate(Constants.kDriveVoltageRampRate);
        sparkMax.burnFlash();
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftLeader = SparkMaxFactory.createDefaultSparkMax(Constants.kLeftDriveLeaderId);
        configureSpark(mLeftLeader, true, true);

        //initialize configured followers
        for(int i = 0; i < Constants.kDriveMotorsPerSide.getMotorsPerSide() -1; i++){
            LazySparkMax tempLeftFollower = SparkMaxFactory.createPermanentFollowerSparkMax(Constants.kLeftDriveFollowerIds[i], mLeftLeader);
            if(tempLeftFollower != null){
                configureSpark(tempLeftFollower, true, false);
                mLeftFollowers.add(tempLeftFollower);
            }

        }

        mRightLeader = SparkMaxFactory.createDefaultSparkMax(Constants.kRightDriveLeaderId);
        configureSpark(mRightLeader, false, true);

        for(int i = 0; i < Constants.kDriveMotorsPerSide.getMotorsPerSide() -1; i++) {
            LazySparkMax tempRightFollower = SparkMaxFactory.createPermanentFollowerSparkMax(Constants.kRightDriveFollowerIds[i], mRightLeader);
            if (tempRightFollower != null){
                configureSpark(tempRightFollower, false, false);
                mRightFollowers.add(tempRightFollower);
            }
        }

        // burn flash so that when spark resets they have the same config
        // mLeftLeader.burnFlash();
        // mLeftFollower.burnFlash();
        // mRightLeader.burnFlash();
        // mRightFollower.burnFlash();

        /*
        mLeftEncoder = mLeftLeader.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);
        mRightEncoder = mRightLeader.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);
        */

        mLeftEncoder = mLeftLeader.getEncoder();
        mRightEncoder = mRightLeader.getEncoder();

        mLeftEncoder.setPositionConversionFactor(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);
        mLeftEncoder.setVelocityConversionFactor(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);
        mRightEncoder.setPositionConversionFactor(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);
        mRightEncoder.setVelocityConversionFactor(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);

        mShifter = new Solenoid(Constants.kPCMId, Constants.kShifterSolenoidId);

        mNavx = new NavX(Constants.kNavXSPIPort);

        // force a solenoid message
        mIsHighGear = false;
        setHighGear(true);

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = true;
        setBrakeMode(false);

        mDriveCurrentLimitState = DriveCurrentLimitState.UNTHROTTLED;
        setDriveCurrentState(DriveCurrentLimitState.THROTTLED, 0.0);
        reloadGains();
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public double left_position_ticks;
        public double right_position_ticks;
        public double left_distance;
        public double right_distance;
        public double left_velocity;
        public double right_velocity;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_velo;
        public double right_velo;
        public double left_feedforward;
        public double right_feedforward;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_voltage = mLeftLeader.getAppliedOutput() * mLeftLeader.getBusVoltage();
        mPeriodicIO.right_voltage = mRightLeader.getAppliedOutput() * mRightLeader.getBusVoltage();

        mPeriodicIO.left_position_ticks = mLeftEncoder.getPosition();
        mPeriodicIO.right_position_ticks = mRightEncoder.getPosition();
        mPeriodicIO.gyro_heading = mNavx.getYaw().rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / Constants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / Constants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

        mPeriodicIO.left_velocity =mLeftEncoder.getVelocity();
        mPeriodicIO.right_velocity =mRightEncoder.getVelocity();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftLeader.set(ControlType.kDutyCycle, mPeriodicIO.left_demand);
            mRightLeader.set(ControlType.kDutyCycle, mPeriodicIO.right_demand);
        } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            mLeftLeader.getPIDController().setReference(mPeriodicIO.left_velo, ControlType.kVelocity, 0);
            mRightLeader.getPIDController().setReference(mPeriodicIO.right_velo, ControlType.kVelocity, 0);
        } else if (mDriveControlState == DriveControlState.PARKING_BRAKE){
            mLeftLeader.set(ControlType.kDutyCycle, mPeriodicIO.left_velo < 0? Constants.kParkingBrakePower:-Constants.kParkingBrakePower);
            mRightLeader.set(ControlType.kDutyCycle, mPeriodicIO.right_velo < 0? Constants.kParkingBrakePower: -Constants.kParkingBrakePower);

        }
        else {
            mLeftLeader.set(ControlType.kDutyCycle, mPeriodicIO.left_demand);
            mRightLeader.set(ControlType.kDutyCycle, mPeriodicIO.right_demand);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(true);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    lastTimeStamp = currentTimeStamp;
                    currentTimeStamp = timestamp;
                    handleFaults();
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case PATH_FOLLOWING:
                            if (mPathFollower != null) {
                                updatePathFollower(timestamp);
                            }
                            break;
                        case DRIVE_TO_STALL:
                            mPeriodicIO.left_demand = Constants.kStallPower;
                            mPeriodicIO.right_demand = Constants.kStallPower;
                            mPeriodicIO.left_feedforward = 0.0;
                            mPeriodicIO.right_feedforward = 0.0;
                            break;
                        default:
                            System.out.println("unexpected drive control state: " + mDriveControlState);
                            break;
                    }

            
                    setDriveCurrentState(DriveCurrentLimitState.UNTHROTTLED, timestamp);
                    
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
    }

    private void handleFaults() {
        for(LazySparkMax rightFollower: mRightFollowers){
            if (rightFollower.getStickyFault(CANSparkMax.FaultID.kHasReset)) {
                System.out.println("Right Follower Reset! Id: " + rightFollower.getDeviceId());
                rightFollower.follow(mRightLeader);
                rightFollower.clearFaults();
            }
        }

        for(LazySparkMax leftFollower: mLeftFollowers){
            if (leftFollower.getStickyFault(CANSparkMax.FaultID.kHasReset)) {
                System.out.println("Left Follower Reset! Id: " + leftFollower.getDeviceId());
                leftFollower.follow(mLeftLeader);
                leftFollower.clearFaults();
            }
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * Constants.kDriveEncoderPPR / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = Kinematics.inverseKinematicsDutyCycle(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }

    public synchronized void autoSteer(double throttle, Optional<AimingParameters> aim_params) {
        if(aim_params.isPresent()) {
            double timestamp = Timer.getFPGATimestamp();
            final double kAutosteerAlignmentPointOffset = 15.0;  // Distance from wall
            boolean reverse = throttle < 0.0;
            boolean towards_goal = reverse == (Math.abs(aim_params.get().getRobotToGoalRotation().getDegrees()) > 90.0);
            Pose2d field_to_vision_target = aim_params.get().getFieldToGoal();
            final Pose2d vision_target_to_alignment_point = Pose2d.fromTranslation(new Translation2d(Math.min(kAutosteerAlignmentPointOffset, aim_params.get().getRange() - kAutosteerAlignmentPointOffset), 0.0));
            Pose2d field_to_alignment_point = field_to_vision_target.transformBy(vision_target_to_alignment_point);
            Pose2d vehicle_to_alignment_point = RobotState.getInstance().getFieldToVehicle(timestamp).inverse().transformBy(field_to_alignment_point);
            Rotation2d vehicle_to_alignment_point_bearing = vehicle_to_alignment_point.getTranslation().direction();
            if (reverse) {
                vehicle_to_alignment_point_bearing = vehicle_to_alignment_point_bearing.rotateBy(Rotation2d.fromDegrees(180.0));
            }
            double heading_error_rad = vehicle_to_alignment_point_bearing.getRadians();

            final double kAutosteerKp = 0.05;
            double curvature = (towards_goal ? 1.0 : 0.0) * heading_error_rad * kAutosteerKp;
            setVelocity(Kinematics.inverseKinematicsVelo(new Twist2d(throttle, 0.0, curvature * throttle * (reverse ? -1.0 : 1.0))));
            setBrakeMode(true);
        }
    }

    /**
     * Configure talons for velocity control
     */
    public synchronized void setVelocity(Kinematics.DriveVelocity velocity) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            setHighGear(true);
            System.out.println("switching to path following");
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            // mLeftLeader.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mRightLeader.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mLeftLeader.configNeutralDeadband(0.0, 0);
            // mRightLeader.configNeutralDeadband(0.0, 0);
        }
        mPeriodicIO.left_velo = velocity.left;
        mPeriodicIO.right_velo = velocity.right;
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            // Plumbed default high.
            mShifter.set(!wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            IdleMode mode = shouldEnable ? IdleMode.kBrake : IdleMode.kCoast;
            mRightLeader.setIdleMode(mode);

            for(LazySparkMax leftFollower: mLeftFollowers){
                leftFollower.setIdleMode(mode);
            }

            for(LazySparkMax rightFollower: mRightFollowers){
                rightFollower.setIdleMode(mode);
            }


            mLeftLeader.setIdleMode(mode);

        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = mNavx.getYaw().inverse();
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized void resetEncoders() {
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        mPeriodicIO = new PeriodicIO();
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / Constants.kDriveEncoderPPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / Constants.kDriveEncoderPPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    /**
     * Gets right drive velocity in inches per second.
     * @return gets right drive velocity.
     */
    public double getRightLinearVelocity() {
        return mPeriodicIO.right_velocity;
    }


    public double getLeftLinearVelocity() {
        return mPeriodicIO.left_velocity;
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
                    new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed,
                            Constants.kMaxLookAheadSpeed),
                    Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                    Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                    Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                    Constants.kPathFollowingProfileKs, Constants.kPathFollowingMaxVel,
                    Constants.kPathFollowingMaxAccel, Constants.kPathFollowingGoalPosTolerance,
                    Constants.kPathFollowingGoalVelTolerance, Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(new Kinematics.DriveVelocity(0, 0));
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!mPathFollower.isFinished()) {

                Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematicsVelo(command);
                setVelocity(setpoint);
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new Kinematics.DriveVelocity(0, 0));
                }
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    private void setDriveLimits(int amps) {
        mRightLeader.setSmartCurrentLimit(amps);
        mLeftLeader.setSmartCurrentLimit(amps);


        for(LazySparkMax leftFollower: mLeftFollowers){
            leftFollower.setSmartCurrentLimit(amps);
        }

        for(LazySparkMax rightFollower: mRightFollowers){
            rightFollower.setSmartCurrentLimit(amps);

        }
    }

    private void setDriveCurrentState(DriveCurrentLimitState desiredState, double timestamp) {
        if (desiredState != mDriveCurrentLimitState && (timestamp - mLastDriveCurrentSwitchTime > 1.0)) {
            mLastDriveCurrentSwitchTime = timestamp;
            System.out.println("Switching drive current limit state: " + desiredState);
            if (desiredState == DriveCurrentLimitState.THROTTLED) {
                setDriveLimits(Constants.kDriveCurrentThrottledLimit);
            } else {
                setDriveLimits(Constants.kDriveCurrentUnThrottledLimit);
            }
            mDriveCurrentLimitState = desiredState;
        }
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        DRIVE_TO_STALL,
        PARKING_BRAKE,
    }

    public enum DriveCurrentLimitState {
        UNTHROTTLED, THROTTLED
    }

    public enum ShifterState {
        FORCE_LOW_GEAR, FORCE_HIGH_GEAR
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);
        setHighGear(true);

        boolean leftSide = SparkMaxChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<CANSparkMax>>() {
                private static final long serialVersionUID = 3643247888353037677L;

                {
                    add(new MotorChecker.MotorConfig<>("left_leader", mLeftLeader));

                    for(LazySparkMax leftFollower: mLeftFollowers){
                        add(new MotorChecker.MotorConfig<>("left_follower", leftFollower));
                    }

                }

            }, new MotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = 3;
                    mRPMFloor = 90;
                    mCurrentEpsilon = 2.0;
                    mRPMEpsilon = 200;
                    mRPMSupplier = mLeftEncoder::getVelocity;
                }
            });
        boolean rightSide = SparkMaxChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<CANSparkMax>>() {
                private static final long serialVersionUID = -1212959188716158751L;

                {
                    add(new MotorChecker.MotorConfig<>("right_leader", mRightLeader));

                    for(LazySparkMax rightFollower: mRightFollowers){
                        add(new MotorChecker.MotorConfig<>("right_follower", rightFollower));;
                    }
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = 5;
                    mRPMFloor = 90;
                    mCurrentEpsilon = 2.0;
                    mRPMEpsilon = 20;
                    mRPMSupplier = mRightEncoder::getVelocity;
                }
            });

        return leftSide && rightSide;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        // SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        // SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        // SmartDashboard.putNumber("X Error", mPeriodicIO.error.getTranslation().x());
        // SmartDashboard.putNumber("Y error", mPeriodicIO.error.getTranslation().y());
        // SmartDashboard.putNumber("Theta Error", mPeriodicIO.error.getRotation().getDegrees());
        SmartDashboard.putNumber("Drive Left MAster NEO out", mLeftLeader.getAppliedOutput());
        for(LazySparkMax tempFollower: mLeftFollowers){
            SmartDashboard.putNumber("Drive left follower NEO out " + tempFollower.getDeviceId(), tempFollower.getAppliedOutput());
        }

        // SmartDashboard.putNumber("Left Voltage Kf", mPeriodicIO.left_voltage / getLeftLinearVelocity());
        // SmartDashboard.putNumber("Right Voltage Kf", mPeriodicIO.right_voltage / getRightLinearVelocity());

        // if (mPathFollower != null) {
        //     SmartDashboard.putNumber("Drive LTE", mPathFollower.getAlongTrackError());
        //     SmartDashboard.putNumber("Drive CTE", mPathFollower.getCrossTrackError());
        // } else {
        //     SmartDashboard.putNumber("Drive LTE", 0.0);
        //     SmartDashboard.putNumber("Drive CTE", 0.0);
        // }

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    public synchronized boolean driveToStall(boolean pushReq, boolean stopReq) {
		startingPush = pushReq && !wasPushing;


		if(startingPush) {
			System.out.println("starting push");
            mDriveControlState = DriveControlState.DRIVE_TO_STALL;
            mPeriodicIO.left_demand = Constants.kStallPower;
            mPeriodicIO.right_demand = Constants.kStallPower;
			pushTimeStamp = currentTimeStamp;
        }
        double leftVelocity = getLeftVelocityInchesPerSec();
        double rightVelocity = getRightVelocityInchesPerSec();

		isTimedOut = (currentTimeStamp - pushTimeStamp > Constants.kStallTimeout);
		finishingPush = (leftVelocity < Constants.kStallSpeed && rightVelocity < Constants.kStallSpeed && currentTimeStamp - pushTimeStamp > Constants.kStallWaitPeriod) || isTimedOut || stopReq;
		wasPushing = pushReq;
		if(leftVelocity < Constants.kStallSpeed && mDriveControlState == DriveControlState.DRIVE_TO_STALL) {
			System.out.println("time to stall " + (currentTimeStamp - pushTimeStamp));
		}

		if(finishingPush && mDriveControlState == DriveControlState.DRIVE_TO_STALL) {
			System.out.println("finishing push");
			System.out.println("speed low? " + (leftVelocity < Constants.kStallSpeed ));
			System.out.println("wait period? " + (currentTimeStamp - pushTimeStamp > Constants.kStallWaitPeriod));
			System.out.println("is timed out? " + isTimedOut);
			mDriveControlState = DriveControlState.OPEN_LOOP;
			pushTimeStamp = 0;
            mPeriodicIO.left_demand = 0;
            mPeriodicIO.right_demand = 0;
			return true;
		}
		return false;

	}

	public void setWantParkingBrake(){
        if(mDriveControlState != DriveControlState.PARKING_BRAKE){
            mDriveControlState = DriveControlState.PARKING_BRAKE;
        }
    }


    public float getWorldLinearAccelX() {
		return mNavx.getWorldLinearAccelX();
	}

	public float getWorldLinearAccelY() {
		return mNavx.getWorldLinearAccelY();
	}

	public float getWorldLinearAccelZ() {
		return mNavx.getWorldLinearAccelZ();
    }

    public double getLeftVelocityInchesPerSec() {
        return mLeftEncoder.getVelocity();
	}
    
    public double getRightVelocityInchesPerSec() {
		return mRightEncoder.getVelocity();
	}

    public synchronized void reloadGains() {
        reloadLowGearPositionGains();
        reloadHighGearVelocityGains();
    }



    /** sets a pid on a motor controller position (high gear high speed)
     *
     * @param motorController to set the pid values on
     */
    public synchronized void reloadHighGearPositionGainsForController(CANSparkMax motorController) {
        motorController.getPIDController().setP(Constants.kDriveHighGearVelocityKp, kHighGearVelocityControlSlot);
        motorController.getPIDController().setI(Constants.kDriveHighGearVelocityKi, kHighGearVelocityControlSlot);
        motorController.getPIDController().setD(Constants.kDriveHighGearVelocityKd, kHighGearVelocityControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveHighGearVelocityKf, kHighGearVelocityControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveHighGearVelocityIZone, kHighGearVelocityControlSlot);

        /*TODO: Do we need this?
		motorController.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, Constants.kDriveTrainPIDSetTimeout);
         */
    }

    /** sets a pid on a motor controller position (high gear low speed)
     *
     * @param motorController to set the pid values on
     */
    public synchronized void reloadHighGearPositionGainsForControllerLowPID(CANSparkMax motorController) {
        motorController.getPIDController().setP(Constants.kDriveHighGearVelocityLowKp, kHighGearVelocityControlSlot);
        motorController.getPIDController().setI(Constants.kDriveHighGearVelocityLowKi, kHighGearVelocityControlSlot);
        motorController.getPIDController().setD(Constants.kDriveHighGearVelocityLowKd, kHighGearVelocityControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveHighGearVelocityLowKf, kHighGearVelocityControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveHighGearVelocityLowIZone, kHighGearVelocityControlSlot);

                /*TODO: Do we need this?
		motorController.configClosedloopRamp(Constants.kDriveHighGearVelocityLowRampRate, Constants.kDriveTrainPIDSetTimeout);
         */
    }

    /** reloads the velocity pid based on whether or not the current wanted pid is high speed or low speed
     *
     */
    public synchronized void reloadHighGearVelocityGains() {
        if (false) {
            System.out.println("low PID");
            reloadHighGearPositionGainsForControllerLowPID(mLeftLeader);
            reloadHighGearPositionGainsForControllerLowPID(mRightLeader);
        } else {
            System.out.println("high PID");
            reloadHighGearPositionGainsForController(mLeftLeader);
            reloadHighGearPositionGainsForController(mRightLeader);
        }
    }

    /** Resets masterLeft and materRight low gear position gains
     *
     */
    public synchronized void reloadLowGearPositionGains() {
        reloadLowGearPositionGainsForController(mLeftLeader);
        reloadLowGearPositionGainsForController(mRightLeader);
    }
    /** sets a pid on a motor controller position (low gear)
     *
     * @param motorController to set the pid values on
     */
    public synchronized void reloadLowGearPositionGainsForController(CANSparkMax motorController) {
        motorController.getPIDController().setP(Constants.kDriveLowGearPositionKp,kLowGearPositionControlSlot);
        motorController.getPIDController().setI(Constants.kDriveLowGearPositionKi,kLowGearPositionControlSlot);
        motorController.getPIDController().setD(Constants.kDriveLowGearPositionKd,kLowGearPositionControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveLowGearPositionKf,kLowGearPositionControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveLowGearPositionIZone,kLowGearPositionControlSlot);
        motorController.getPIDController().setSmartMotionMaxVelocity(Constants.kDriveLowGearMaxVelocity,kLowGearPositionControlSlot);
        motorController.getPIDController().setSmartMotionMaxAccel(Constants.kDriveLowGearMaxAccel, kLowGearPositionControlSlot);
        /*TODO:DO we need this?
		motorController.configClosedloopRamp(Constants.kDriveLowGearPositionRampRate, Constants.kDriveTrainPIDSetTimeout);
		*/

    }
}