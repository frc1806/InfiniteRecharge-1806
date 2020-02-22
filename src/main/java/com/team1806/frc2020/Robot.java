package com.team1806.frc2020;

import com.team1806.frc2020.auto.AutoModeExecutor;
import com.team1806.frc2020.auto.modes.AutoModeBase;
import com.team1806.frc2020.controlboard.ControlBoard;
import com.team1806.frc2020.controlboard.GamepadButtonControlBoard;
import com.team1806.frc2020.controlboard.IControlBoard;
import com.team1806.frc2020.game.Shot;
import com.team1806.frc2020.loops.Looper;
import com.team1806.frc2020.subsystems.*;
import com.team1806.lib.geometry.Pose2d;
import com.team1806.lib.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import com.team1806.lib.util.*;
import com.team1806.lib.vision.AimingParameters;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard = ControlBoard.GetInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // subsystems
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Jetson mJetson;


    // button placed on the robot to allow the drive team to zero the robot right
    // before the start of a match
    DigitalInput resetRobotButton = new DigitalInput(Constants.kResetButtonChannel);

    private boolean mHasBeenEnabled = false;

    private LatchedBoolean mShootPressed = new LatchedBoolean();
    private LatchedBoolean mThrustReleased = new LatchedBoolean();
    private LatchedBoolean mThrustPressed = new LatchedBoolean();
    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();
    private LatchedBoolean mAutoSteerPressed = new LatchedBoolean();
    private boolean mStickyShoot;

    private boolean bAutoModeStale = false;
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;
    public static AutoModeBase selectedAuto;
    private String selectedModeName;
    private String lastSelectedModeName;
    private boolean mDriveByCameraInAuto = false;

    Robot() {
        CrashTracker.logRobotConstruction();
        Jetson.JetsonConstants jetsonConstants = new Jetson.JetsonConstants();
        jetsonConstants.kTable = "Vision";
        jetsonConstants.OffsetFromTurret = new Pose2d(0.0, 5.0, Rotation2d.fromDegrees(0.0));
        jetsonConstants.kHorizontalPlaneToLens = new Rotation2d(Rotation2d.fromDegrees(0.0));
        mJetson = new Jetson(jetsonConstants);

    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                    mRobotStateEstimator,
                    mDrive,
                    mInfrastructure,
                    mJetson,
                    Flywheel.GetInstance(),
                    Turret.GetInstance(),
                    Hood.GetInstance(),
                    ColorWheelReader.GetInstance(),
                    Conveyor.GetInstance(),
                    mSuperstructure);
            AutoModeSelector.registerDisabledLoop(mDisabledLooper);
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            AutoModeSelector.initAutoModeSelector();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            // Reset all auto mode state.
            if(mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
                selectedAuto = AutoModeSelector.getSelectedAutoMode();
            }

            mInfrastructure.setIsManualControl(false);

            mDisabledLooper.start();

            mDrive.setBrakeMode(false);
            mThrustReleased.update(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        mAutoModeExecutor = new AutoModeExecutor();
        bAutoModeStale = true;
        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());
            mHasBeenEnabled = true;

            mInfrastructure.setIsManualControl(true); // turn on compressor when superstructure is not moving

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            mAutoModeExecutor.setAutoMode(selectedAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mHasBeenEnabled = true;

            mEnabledLooper.start();

            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }


    @Override
    public void disabledPeriodic() {
        try {
            if (!resetRobotButton.get() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!");
            }

            selectedModeName = SmartDashboard.getString(
                AutoModeSelector.SELECTED_AUTO_MODE_DASHBOARD_KEY,
                "com.team1806.frc2020.auto.modes.DoNothingMode");
      if(!selectedModeName.equals(lastSelectedModeName) || bAutoModeStale){
          bAutoModeStale = false;
          selectedAuto = AutoModeSelector.getSelectedAutoMode();
      }
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        boolean signalToResume = !mControlBoard.getWantsLowGear();
        boolean signalToStop = mControlBoard.getWantsLowGear();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            mAutoModeExecutor.interrupt();
        }

        if (mDriveByCameraInAuto || mAutoModeExecutor.isInterrupted()) {
            manualControl();
        }
    }

    @Override
    public void teleopPeriodic() {
        try {
            manualControl();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void manualControl() {
        double timestamp = Timer.getFPGATimestamp();
        boolean rumble = false;
        double throttle = mControlBoard.getThrottle();

        Optional<AimingParameters> drive_aim_params = RobotState.getInstance().getAimingParameters(-1, Constants.kMaxGoalTrackAge);

        boolean wantsLowGear = mControlBoard.getWantsLowGear();
        // drive

        if(mControlBoard.getWantsAutoSteer()){
            mDrive.autoSteer(throttle, drive_aim_params);
        }
        else if(mControlBoard.getWantsPark()){
            mDrive.setHighGear(false);
            mDrive.setWantParkingBrake();
        } else{
            mDrive.setHighGear(!wantsLowGear);
            mDrive.setCheesyishDrive(throttle, -mControlBoard.getTurn(), mControlBoard.getQuickTurn());
        }


        if (mControlBoard.getShoot()){
            mSuperstructure.setWantShot(Shot.STRAIGHT_ON_AUTOLINE);
        }
        else if (mControlBoard.getWantDashboardShot()){
            NetworkTable table = NetworkTableInstance.getDefault().getTable("Shot");
            double turretAngle = table.getEntry("Turret").getDouble(0);
            double hoodAngle = table.getEntry("Hood").getDouble(0);
            double flywheelRPM = table.getEntry("FlywheelRPM").getDouble(0);
            mSuperstructure.setWantShot(new Shot(turretAngle, hoodAngle, flywheelRPM));
        }

        else if(mControlBoard.getAutoLineShot()){
            mSuperstructure.setWantShot(Shot.STRAIGHT_ON_AUTOLINE);
        }
        else if(mControlBoard.getCloseShot()){
            mSuperstructure.setWantShot(Shot.CLOSE_SHOT);
        }
        else if (mControlBoard.getLongShot()){
            mSuperstructure.setWantShot(Shot.STRAIGHT_ON_FROM_MAX_DIST);
        }
        else if (mControlBoard.getTrenchShot()){
            mSuperstructure.setWantShot(Shot.FROM_TRENCH);
        }
        else if (mControlBoard.getWantsFrontIntake()){
            mSuperstructure.frontIntake();
        }
        else if (mControlBoard.getWantsRearIntake()){
            mSuperstructure.backIntake();
        }
        else{
            mSuperstructure.stop();
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void endCompetition() { }
}
