package com.team1806.frc2020.paths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team1806.frc2020.Constants;
import com.team1806.lib.control.Path;
import com.team1806.frc2020.Robot;
import com.team1806.frc2020.RobotState;
import com.team1806.frc2020.auto.modes.VisionMode;
import com.team1806.frc2020.subsystems.Drive;
import com.team1806.lib.geometry.Pose2d;
import com.team1806.lib.geometry.Rotation2d;
import com.team1806.lib.geometry.Translation2d;

import java.util.ArrayList;

public class VisionPath implements PathContainer {
    public enum BayLocation {
        CARGO_SHIP_SIDE_1_AUDIANCE(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0))),
        CARGO_SHIP_SIDE_2_AUDIANCE(new Pose2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_3_AUDIANCE(new Pose2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_1_NONAUDIANCE(new Pose2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_2_NONAUDIANCE(new Pose2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_3_NONAUDIANCE(new Pose2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_FRONT_AUDIANCE(new Pose2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_FRONT_NONAUDIANCE(new Pose2d(new Translation2d(), new Rotation2d()));
        public Pose2d getBayCoordinates() {
            return bayCoordinates;
        }

        private Pose2d bayCoordinates;
        BayLocation(Pose2d bayCoordinates) {
            this.bayCoordinates = bayCoordinates;
        }
    }

    BayLocation trackedBay = BayLocation.CARGO_SHIP_FRONT_AUDIANCE;
    Pose2d odometry;
    public final int speed = 90;
    public Pose2d bayyPose;
    public Double targetsTimestamp;
    public VisionPath(Pose2d odo) {
        odometry = odo;
    }
    @Override
    public Path buildPath() {
        double fps = 1;
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        if(false) {
            Pose2d roboPose = generateTemporaryVisionPose();
            sWaypoints.add(new PathBuilder.Waypoint(roboPose.getTranslation().x(), roboPose.getTranslation().y(), 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, roboPose.getTranslation().y()- 4, 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, 20, 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, 18, 0, speed));
        }
        else {
                Pose2d roboPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
                System.out.println("Field to vehicle: (" + roboPose.getTranslation().x() + ", " + roboPose.getTranslation().y() + ")");
                //Pose2d bayyPose = new Pose2d(new Translation2d(roboPose.getTranslation().x() - odometry.getTranslation().x(), roboPose.getTranslation().y() + odometry.getTranslation().y()), Rotation2d.fromDegrees(roboPose.getRotation().getDegrees() + odometry.getRotation().getDegrees()));
                bayyPose = generateBayVisionPoseFromODO();
                double robotEndDistanceFromTarget = -13.75;
                double robotDistanceToEnd = -interpolateAlongLine(bayyPose.getTranslation(), robotEndDistanceFromTarget, bayyPose.getRotation().getRadians()).subtract(roboPose.getTranslation()).norm();
                double secondFromEndDistance = Math.min((robotDistanceToEnd+robotEndDistanceFromTarget) * 0.70, robotEndDistanceFromTarget - 4);
                double firstFromEndDistance = Math.min((robotDistanceToEnd+robotEndDistanceFromTarget) * 0.30, robotEndDistanceFromTarget - 2);
                Drive driveTrain = Drive.getInstance();
                double averageVelocity = (driveTrain.getLeftVelocityInchesPerSec() + driveTrain.getRightVelocityInchesPerSec()) / 2;
                sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(roboPose.getTranslation(), 0, roboPose.getRotation().getRadians()), 0, Math.max(10, averageVelocity)));
                if(robotDistanceToEnd > 40){

                    //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(roboPose.getTranslation(), 2, roboPose.getRotation().getRadians()), 0, Math.max(10, averageVelocity)));
                    Translation2d nonSideInterpolSecondPoint = interpolateAlongLine(bayyPose.getTranslation(), secondFromEndDistance,  bayyPose.getRotation().getRadians());
                    sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(nonSideInterpolSecondPoint,roboPose.getTranslation().subtract(nonSideInterpolSecondPoint).norm()*0.25 , roboPose.getRotation().getRadians()),Math.abs(firstFromEndDistance - robotEndDistanceFromTarget) *0.25, speed));

                }
                sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose.getTranslation(), firstFromEndDistance,  bayyPose.getRotation().getRadians()),Math.abs(firstFromEndDistance - robotEndDistanceFromTarget) *0.5, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(interpolateAlongLine(bayyPose.getTranslation(), -27, bayyPose.getRotation().getRadians()), 3, roboPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -26, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -23, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -21, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -19, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose.getTranslation(), -13.5, bayyPose.getRotation().getRadians()), 0, speed));
                sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose.getTranslation(), robotEndDistanceFromTarget, bayyPose.getRotation().getRadians()), 0, speed));

        }
        return PathBuilder.buildPathFromWaypoints(sWaypoints);

    }
    public Pose2d generateTemporaryVisionPose() {
        double dist = SmartDashboard.getNumber("Vdistance", 0);
        double angle = SmartDashboard.getNumber("Vangle", 0);

        double x = dist * Math.sin(angle);
        double y = dist * Math.cos(angle);

        return new Pose2d(new Translation2d(x,y), Rotation2d.fromDegrees(angle));
    }
    public Pose2d generateBayVisionPoseFromODO() {
        return RobotState.getInstance().getFieldToVisionTarget();
        
//        Target goalTarget = targets.get(0);
//        if(true) {
//            for(int i = 1; i < targets.size(); i++){
//                if(targets.get(i).getDistance() < goalTarget.getDistance()){
//                    goalTarget = targets.get(i);
//                }
//            }
//        }
//        else {
//            for(int i = 1; i < targets.size(); i++){
//                if(targets.get(i).getMiddle() > goalTarget.getMiddle()){
//                    goalTarget = targets.get(i);
//                }
//            }
//        }
//        VisionMode.mAngle = goalTarget.getTargetHeadingOffset();
//        Pose2d bayPose = new Pose2d();
//        if(targets.size() != 0) {
//            Pose2d robotPose = RobotState.getInstance().getFieldToVehicle(targetsTimestamp - Constants.kVisionExpectedCameraLag);
//            double goalHeading = robotPose.getRotation().getDegrees() - goalTarget.getTargetHeadingOffset();
//            Pose2d xCorrectedRobotPose = interpolateAlongLine(robotPose, -2, robotPose.getRotation().getRadians(), robotPose.getRotation().getRadians());
//            Pose2d correctedRobotPose = interpolateAlongLine(xCorrectedRobotPose, -8.5, robotPose.getRotation().getRadians() + Math.toRadians(90), Rotation2d.fromRadians(robotPose.getRotation().getRadians()).getRadians());
//            bayPose = interpolateAlongLine(correctedRobotPose, goalTarget.getDistance(), Math.toRadians(-goalTarget.getRobotToTarget()+ robotPose.getRotation().getDegrees()), Math.toRadians(-goalTarget.getTargetHeadingOffset() + robotPose.getRotation().getDegrees()));
//        }

        //return null;
    }

    public Translation2d interpolateAlongLine(Translation2d point, double adjust, double heading) {
        double x = 0;
        double y = 0;
            x = point.x() + adjust * Math.cos(heading);
            y = point.y() + adjust * Math.sin(heading);
/*        }
        else if (heading >= Math.toRadians(90) && heading < Math.toRadians(180) ){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(180) && heading < Math.toRadians(270)) || (heading>= Math.toRadians(-180) && heading < Math.toRadians(-90))){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * -Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(270) && heading < Math.toRadians(360)) || (heading>= Math.toRadians(-90) && heading < 0)){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else{
            System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAA VISION PATH.java : interpolateAlongLing : Angle didn't fit into defined quadrants");
        }
*/        return new Translation2d(x,y);

    }
    public Pose2d interpolateAlongLine(Pose2d point, double adjust, double heading, double heading2) {
        double x = 0;
        double y = 0;
            x = point.getTranslation().x() + adjust * Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
/*        }
        else if (heading >= Math.toRadians(90) && heading < Math.toRadians(180) ){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(180) && heading < Math.toRadians(270)) || (heading>= Math.toRadians(-180) && heading < Math.toRadians(-90))){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * -Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(270) && heading < Math.toRadians(360)) || (heading>= Math.toRadians(-90) && heading < 0)){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else{
            System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAA VISION PATH.java : interpolateAlongLing : Angle didn't fit into defined quadrants");
        }
*/
        return new Pose2d(new Translation2d(x,y), Rotation2d.fromRadians(heading2));

    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
