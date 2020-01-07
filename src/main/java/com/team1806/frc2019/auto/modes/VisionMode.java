package com.team1806.frc2019.auto.modes;

import com.team1806.frc2019.auto.AutoModeEndedException;
import com.team1806.frc2019.RobotState;
import com.team1806.frc2019.auto.actions.DriveToStall;
import com.team1806.frc2019.auto.actions.DrivePathAction;
import com.team1806.frc2019.paths.VisionPath;
import com.team1806.frc2019.subsystems.Drive;
import com.team1806.lib.geometry.Pose2d;
import com.team1806.lib.geometry.Rotation2d;
import com.team1806.lib.geometry.Translation2d;

public class VisionMode extends AutoModeBase {
    public static double mAngle = -1000;
    boolean done = false;
    @Override
    protected void routine() throws AutoModeEndedException {
        done = false;
        int testX = 72;
        int testY = 12;
        double testAngle = 0;
        Drive.getInstance().stop();
        VisionPath visPath = new VisionPath(new Pose2d(new Translation2d(testX, testY), Rotation2d.fromRadians(testAngle)));
        runAction(new DrivePathAction(visPath));
        System.out.println("heading after path " + RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees());
       // runAction(new TurnTowardsPoint(visPath.bayyPose.getTranslation()));
        System.out.println("heading after TTP " + RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees());
        runAction(new DriveToStall());
        done = true;
       // runAction(new WaitAction(15));

    }

    public boolean getIsDone(){
        return done;
    }
}
