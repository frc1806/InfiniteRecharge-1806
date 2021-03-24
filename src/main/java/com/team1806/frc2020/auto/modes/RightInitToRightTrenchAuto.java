package com.team1806.frc2020.auto.modes;

import com.team1806.frc2020.RobotState;
import com.team1806.frc2020.auto.AutoModeEndedException;
import com.team1806.frc2020.auto.actions.DrivePathAction;
import com.team1806.frc2020.auto.actions.WaitAction;
import com.team1806.frc2020.paths.RightInitToRightTrench;
import com.team1806.lib.geometry.Pose2d;
import com.team1806.lib.geometry.Rotation2d;
import com.team1806.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class RightInitToRightTrenchAuto extends AutoModeBase {
    protected void routine() throws AutoModeEndedException {
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), new Pose2d(new Translation2d(131, 250), Rotation2d.fromDegrees(180.0)));
        runAction(new DrivePathAction(new RightInitToRightTrench(), true));
        runAction(new WaitAction(15));
    }
}
