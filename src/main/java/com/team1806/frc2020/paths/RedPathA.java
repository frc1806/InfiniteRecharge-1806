package com.team254.frc2017.paths;
import java.util.ArrayList;
import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;
public class UntitledPath implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(106,90,0,0));
        sWaypoints.add(new Waypoint(130,75,20,60));
        sWaypoints.add(new Waypoint(166,60,5,60));
        sWaypoints.add(new Waypoint(186,75,5,60));
        sWaypoints.add(new Waypoint(193,95,5,60));
        sWaypoints.add(new Waypoint(196,115,0,60));
        sWaypoints.add(new Waypoint(250,115,0,60));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(106, 90), Rotation2d.fromDegrees(180.0));
    }
    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":106,"y":90},"speed":0,"radius":0,"comment":""},{"position":{"x":130,"y":75},"speed":60,"radius":20,"comment":""},{"position":{"x":166,"y":60},"speed":60,"radius":5,"comment":""},{"position":{"x":186,"y":75},"speed":60,"radius":5,"comment":""},{"position":{"x":193,"y":95},"speed":60,"radius":5,"comment":""},{"position":{"x":196,"y":115},"speed":60,"radius":0,"comment":""},{"position":{"x":250,"y":115},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath
}