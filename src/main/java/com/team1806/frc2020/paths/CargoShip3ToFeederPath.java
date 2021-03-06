package com.team1806.frc2020.paths;

import com.team1806.lib.control.Path;

import java.util.ArrayList;

public class CargoShip3ToFeederPath implements PathContainer {
    public static final String kLookForTargetMarker = "LOOK_FOR_TARGET";

    boolean mLeft;

    public CargoShip3ToFeederPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(250, (mLeft ? 1.0 : -1.0) * 5, 0, 120));
        sWaypoints.add(new PathBuilder.Waypoint(105, (mLeft ? 1.0 : -1.0) * 40, 35, 100));
        sWaypoints.add(new PathBuilder.Waypoint(55, (mLeft ? 1.0 : -1.0) * 80, 0, 120, kLookForTargetMarker));
        sWaypoints.add(new PathBuilder.Waypoint(-20, (mLeft ? 1.0 : -1.0) * 80, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
