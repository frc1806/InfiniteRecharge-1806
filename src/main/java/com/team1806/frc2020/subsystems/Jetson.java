package com.team1806.frc2020.subsystems;

import com.team1806.frc2020.RobotState;
import com.team1806.lib.geometry.Pose2d;
import com.team1806.lib.geometry.Rotation2d;
import com.team1806.lib.geometry.Translation2d;
import com.team1806.lib.vision.GoalTrack;
import com.team1806.lib.vision.GoalTracker;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class Jetson extends Subsystem {
    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    enum Pipeline{

        kGoalPipeline(0);

        private int mPipeNum;
        Pipeline(int pipeNum){
            mPipeNum = pipeNum;
        }
        public int getPipeNumber(){
            return mPipeNum;
        }
    }

    public static class JetsonConstants{
        public String kName = "";
        public String kTable = "";
        public double kHeight = 0.0;
        public Pose2d OffsetFromTurret = Pose2d.identity();
        public Rotation2d kHorizontalPlaneToLens = Rotation2d.identity();
    }

    private NetworkTable mNetworkTable;
    private JetsonConstants mConstants = null;
    private PeriodicIO mPeriodicIO;

    public Jetson(JetsonConstants constants){
        mConstants = constants;
        mNetworkTable = (NetworkTableInstance.getDefault().getTable(constants.kTable));
        mPeriodicIO = new PeriodicIO();
    }

    public static class PeriodicIO{
        //INPUTS
        public double latency;
        public double distanceToTarget;
        public double robotToTarget;
        public double targetSkew;
        //OUTPUTS
        public Pipeline wantedPipeline;

    }

    @Override
    public void writePeriodicOutputs(){

    }

    @Override
    public void readPeriodicInputs(){
        double processTime = mNetworkTable.getEntry("VisionProcessTime").getDouble(0);
        double distanceToTarget = mNetworkTable.getEntry("TargetDistance").getDouble(0);
        double targetSkew = mNetworkTable.getEntry("TargetSkew").getDouble(0);
        double robotToTarget = mNetworkTable.getEntry("RobotToTarget").getDouble(0);
        if(processTime != 0.0 && distanceToTarget != 0.0 && targetSkew != 0.0 && robotToTarget != 0.0)
        {
            mPeriodicIO.distanceToTarget = distanceToTarget;
            mPeriodicIO.robotToTarget = robotToTarget;
            mPeriodicIO.targetSkew = targetSkew;
            mPeriodicIO.latency = processTime;
            double trackedTime = Timer.getFPGATimestamp() - (processTime * 1e-9);
            ArrayList<Pose2d> targets = new ArrayList<>();
            targets.add(getTargetPosition(RobotState.getInstance().getFieldToVehicle(trackedTime)));
            RobotState.getInstance().addVisionUpdate(trackedTime, targets);
        }
    }

    private Pose2d getTargetPosition(Pose2d robotPosition){
        double angleForConversion = robotPosition.getRotation().getDegrees() + mPeriodicIO.robotToTarget + mConstants.kHorizontalPlaneToLens.getDegrees();
        double targetX = mPeriodicIO.distanceToTarget * Math.cos(Math.toRadians(angleForConversion)) + mConstants.OffsetFromTurret.getTranslation().x();
        double targetY = mPeriodicIO.distanceToTarget * Math.sin(Math.toRadians(angleForConversion)) + mConstants.OffsetFromTurret.getTranslation().y();
        double targetHeading = robotPosition.getRotation().getDegrees() + mPeriodicIO.robotToTarget + mPeriodicIO.targetSkew;
        return new Pose2d(new Translation2d(targetX,targetY), new Rotation2d().fromDegrees(targetHeading));
    }

}
