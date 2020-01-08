package com.team1806.frc2020.paths;

import com.team1806.frc2020.paths.PathBuilder.Waypoint;
import com.team1806.lib.control.Path;

import java.util.ArrayList;

public class Hab1ToRocketFarPath implements PathContainer {
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";

    boolean mLeft;

    public Hab1ToRocketFarPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(20, 0, 0, 40.0));
        sWaypoints.add(new Waypoint(170, 0, 83.0, 100.0, kStartAutoAimingMarker));
        sWaypoints.add(new Waypoint(205, (mLeft ? 1.0 : -1.0) * 83, 0, 100.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}