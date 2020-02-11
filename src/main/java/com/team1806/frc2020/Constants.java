package com.team1806.frc2020;

import com.team1806.lib.drivers.LazySparkMax;
import com.team1806.lib.geometry.Pose2d;
import com.team1806.lib.geometry.Rotation2d;
import com.team1806.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.SPI;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    // wheels
    // Tuned for a robot we don't have
    public static final double kDriveWheelTrackWidthInches = 25.42;
    public static final double kDriveWheelDiameterInches = 3.938;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0469745223;

    // tuned dynamics
    public static final double kDriveLinearVIntercept = 0.1801; // V
    public static final double kDriveLinearKv = 0.0919; // V per rad/s
    public static final double kDriveLinearKa = 0.03344; // V per rad/s^2
    public static final double kDriveAngularKa = 0.02897 * 2.0; // V per rad/s^2
    public static final double kRobotLinearInertia = 63.9565; // kg
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia;  // kg m^2
    public static final double kRobotAngularDrag = 0.0; // N*m / (rad/sec)

    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    public static final double kBumperHeight = 6.6 + 2.0; // inches to ground + 2 in buffer

    public static final double kCollisionJerkThreshold = 1.0;
    
    //driveToStall
    public final static double kStallTimeout = 2;
    public final static double kStallWaitPeriod = .3;
    public final static double kStallSpeed = 500;
    public final static double kStallPower = .15;


    //Camera
    public static final double kCameraFrameRate = 60.0;
    public static final double kMaxGoalTrackSmoothingTime = 2.0;
    public static final double kMaxGoalTrackAge = 1.0;
    public static final double kMaxTrackerDistance = 115.0;
    public static final Rotation2d kHorizontalPlaneToCameraAngle = new Rotation2d(3.1415/4.0, false);
    public static final Pose2d kCameraToRobotCenterOffset = new Pose2d(new Translation2d(8.0, 11.0), new Rotation2d(0, false));
    public static final double kCameraLensHeight = 55.0;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final double kPortTargetHeight = 39.125;
    public static final double kHatchTargetHeight = 31.5;

    //NavX
    public static final SPI.Port kNavXSPIPort = SPI.Port.kMXP;


    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here: TODO Update for 2020
    // https://docs.google.com/spreadsheets/d/1w9V3_tqQ0npdc9U8WPD-6zJkKouunKzHvPXLbHEWwxk/edit#gid=0

    // drive
    public static final int kLeftDriveLeaderId = 1;
    public static final int kRightDriveLeaderId = 5;

    public static final Integer[] kLeftDriveFollowerIds = {2, 3, 4};
    public static final Integer[] kRightDriveFollowerIds = {6, 7, 8};



    public static final double kDriveEncoderPPR = 1000.0;

    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 12.0; // inches per second
    public static final double kMaxLookAhead = 48.0; // inches
    public static final double kMaxLookAheadSpeed = 120.0; // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    public static final double kPathFollowingProfileKp = 0.3 / 12.0;  // % throttle per inch of error
    public static final double kPathFollowingProfileKi = 0.0;
    public static final double kPathFollowingProfileKv = 0.01 / 12.0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKffv = 0.003889;  // % throttle per inch/s
    public static final double kPathFollowingProfileKffa = 0.001415;  // % throttle per inch/s^2
    public static final double kPathFollowingProfileKs = 0.1801 / 12.0;  // % throttle
    public static final double kPathFollowingGoalPosTolerance = 3.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps

    public static final double kFollowTargetSamplingDistance = 1.0;
    public static final double kFollowTargetLookahead = 30.0;
    public static final double kFollowTargetTolerance = 0.1;
    public static final double kFollowTargetSteeringScale = 0.05 * 24;
    public static final double kFollowTargetMaxThrottle = 0.8;
    public static final double kFollowTargetExtraWaypointDistance = 30.0;
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    //
    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in counts per tenth of a second
    public final static double kDriveHighGearVelocityKp = .001; //.0004;//.08;//.16; //1.01;
    public final static double kDriveHighGearVelocityKi = 0.00000001;
    public final static double kDriveHighGearVelocityKd =  0.002; //.6125; //1.25; //7.8; //0.0001; //6.0/1500;
    public final static double kDriveHighGearVelocityKf = 0.00004;//.0175; //.035; //0.21; //.025;
    public final static int kDriveHighGearVelocityIZone = 0;
    public final static double kDriveHighGearVelocityRampRate = .1;
    public final static double kDriveHighGearNominalOutput = 0.25;
    public final static double kDriveHighGearMaxSetpoint = 12 * 13; //FPS

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in counts per tenth of a second
    public final static double kDriveHighGearVelocityLowKp = .0004; // 1.2/1500;
    public final static double kDriveHighGearVelocityLowKi = 0.0000000; //0.0;
    public final static double kDriveHighGearVelocityLowKd = 0; //0.0001; //6.0/1500;
    public final static double kDriveHighGearVelocityLowKf = 0.00000254;//0; //.025;
    public final static int kDriveHighGearVelocityLowIZone = 0;
    public final static double kDriveHighGearVelocityLowRampRate = .1;
    public final static double kDriveHighGearLowNominalOutput = 0.25;
    public final static double kDriveHighGearLowMaxSetpoint = 10.5 * 12; //FPS

    // PID gains for drive velocity loop ***This should be high gear lollz sorry yall
    // This is typically used in the most in the TurnToPoint action
    // Units: setpoint, error, and output are in counts
    public final static double kDriveLowGearPositionKp = .1; //.008
    public final static double kDriveLowGearPositionKi = .00044;
    public final static double kDriveLowGearPositionKd = 0.1; //0.0035
    public final static double kDriveLowGearPositionKf = 0.000;
    public final static double kDriveLowGearPositionIZone = 1.5;
    public final static int kDriveLowGearMaxVelocity = 700; // Counts
    public final static int kDriveLowGearMaxAccel = 20; // Counts
    public final static double kDriveTurnMaxPower = .6;

    public static final double kWristToBottomEndEffectorLength = 15.91; // Length (in) from wrist joint to bottom of end effector
    public static final double kEndEffectorBottomAngle = 7.75; // Angle (°) from wrist joint to bottom of end effector when wrist is at 0°


    // reset button
    public static final int kResetButtonChannel = 4;

    public static final boolean kUseDriveGamepad = true;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.2;

    // solenoids
    public static final int kPCMId = 0;
    public static final int kShifterSolenoidId = 1;
    public static final int kFrontIntakeFowardChannel = 4;
    public static final int kFrontIntakeReverseChannel = 5;
    public static final int kBackIntakeFowardChannel = 6;
    public static final int kBackIntakeReverseChannel = 7;

    // flywheel
    public static final int kFlywheelSparkMaxLeader = 30;
    public static final int kFlywheelSparkMaxFollower = 31;
    public static final double kFlywheelSpeedControlkp = 1;
    public static final double kFlywheelSpeedControlki = 0;
    public static final double kFlywheelSpeedControlkd = 0;
    public static final double kFlywheelSpeedControlkf = 0;
    public static final double kFlywheelAcceptableSpeedRange = 100;
    public static final double kFlywheelAcceptableAccleration = 100;
    public static final double kFlywheelGearScalingFactor = 35.0 / 18.0;

    // turret
    public static final int kTurretMotorId = 60;
    public static final double kTurretDegreesPerCount = 0;
    public static final double kTurretPositionControlKp = 0;
    public static final double kTurretPositionControlKi = 0;
    public static final double kTurretPositionControlKd = 0;
    public static final double kTurretPositionControlKf = 0;
    public static final double kTurretPositionMin = 0;
    public static final double kTurretPositionMax = 0;
    public static final double kTurretAcceptableAngleDeviation = 0;
    public static final double kTurretAcceptableSpeed = 0;

    //hood
    public static final int kHoodMotorId = 61;
    public static final double kHoodDegreesPerCount = 0;
    public static final double kHoodPositionControlKp = 0;
    public static final double kHoodPositionControlKi = 0;
    public static final double kHoodPositionControlKd = 0;
    public static final double kHoodPositionControlKf = 0;
    public static final double kHoodPositionMin = 0;
    public static final double kHoodPositionMax = 0;
    public static final double kHoodAcceptableAngleDeviation = 0;
    public static final double kHoodAcceptableSpeed = 0;


    //Conveyor
    public static final int kTriggerConveyorMotorId = 10;
    public static final int kTopConveyorMotorId = 11;
    public static final int kBottomConveyorMotorId = 12;

    public static final int kOuterIntake = 13;

    public static final double kNEORPMToControlPanelRPMConversionFactor = 1.0;

    public static final double kTriggerConveyorVelocityControlKp = 0;
    public static final double kTriggerConveyorVelocityControlKi = 0;
    public static final double kTriggerConveyorVelocityControlKd = 0;
    public static final double kTriggerConveyorVelocityControlKf = 0;

    public static final double kTopConveyorVelocityControlKp = 0;
    public static final double kTopConveyoVelocityControlKi = 0;
    public static final double kTopConveyorVelocityControlKd = 0;
    public static final double kTopConveyorVelocityControlKf = 0;

    public static final double kBottomConveyorVelocityControlKp = 0;
    public static final double kBottomConveyorVelocityControlKi = 0;
    public static final double kBottomConveyorVelocityControlKd = 0;
    public static final double kBottomConveyorVelocityControlKf = 0;

    public static final double kOuterIntakeVelocityControlKp = 0;
    public static final double kOuterIntakeVelocityControlKi = 0;
    public static final double kOuterIntakeVelocityControlKd = 0;
    public static final double kOuterIntakeVelocityControlKf = 0;

    public static final double kTriggerLaunchSpeed = 10000.0;
    public static final double kBottomConveyorSpeed = 100;
    public static final double kTopConveyorSpeed = 100;


    public enum DriveMotorsPerSide{

        ONE(1),
        TWO(2),
        THREE(3),
        FOUR(4),
        FIVE(5);

        private int mMotorsPerSide;

        private DriveMotorsPerSide(int motorsPerSide){
            mMotorsPerSide = motorsPerSide;
        }
        public int getMotorsPerSide(){
            return mMotorsPerSide;
        }
    };

    public static final DriveMotorsPerSide kDriveMotorsPerSide = DriveMotorsPerSide.TWO;


    /*
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}
