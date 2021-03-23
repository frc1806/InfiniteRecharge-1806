package com.team1806.frc2020.paths;

import com.team1806.frc2020.paths.PathBuilder.Waypoint;
import com.team1806.lib.control.Path;

import java.util.ArrayList;

public class BluePathB implements PathContainer {
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";
    public static final String kStartRaisingElevatorMarker = "START_RAISING_ELEVATOR";


    public BluePathB() {
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(45, 30, 0, 0));
        sWaypoints.add(new Waypoint(180, 60, 15, 60));
        sWaypoints.add(new Waypoint(200, 80, 15, 60));
        sWaypoints.add(new Waypoint(240, 120, 15, 60));
        sWaypoints.add(new Waypoint(260, 140, 15, 60));
        sWaypoints.add(new Waypoint(300, 60, 15, 60));
        sWaypoints.add(new Waypoint(335, 50, 15, 60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
