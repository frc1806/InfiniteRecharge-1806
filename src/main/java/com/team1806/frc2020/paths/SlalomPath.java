package com.team1806.frc2020.paths;

import com.team1806.frc2020.paths.PathBuilder.Waypoint;
import com.team1806.lib.control.Path;

import java.util.ArrayList;

public class SlalomPath implements PathContainer {
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";
    public static final String kStartRaisingElevatorMarker = "START_RAISING_ELEVATOR";

    public SlalomPath() {
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(44,30,0,0));
        sWaypoints.add(new Waypoint(120,100,15,60));
        sWaypoints.add(new Waypoint(240,120,15,60));
        sWaypoints.add(new Waypoint(280,60,15,60));
        sWaypoints.add(new Waypoint(300,50,15,60));
        sWaypoints.add(new Waypoint(320,60,15,60));
        sWaypoints.add(new Waypoint(300,110,15,60));
        sWaypoints.add(new Waypoint(270,60,15,60));
        sWaypoints.add(new Waypoint(210,40,15,60));
        sWaypoints.add(new Waypoint(120,40,15,60));
        sWaypoints.add(new Waypoint(43,85,15,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
