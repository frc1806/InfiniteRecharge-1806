package com.team1806.frc2020.paths;

import com.team1806.frc2020.paths.PathBuilder.Waypoint;
import com.team1806.lib.control.Path;

import java.util.ArrayList;

public class PathBarrelRace implements PathContainer {
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";
    public static final String kStartRaisingElevatorMarker = "START_RAISING_ELEVATOR";

    public PathBarrelRace() {

    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(45,80,0,0));
        sWaypoints.add(new Waypoint(80,80,15,60));
        sWaypoints.add(new Waypoint(180,60,15,60));
        sWaypoints.add(new Waypoint(160,40,15,60));
        sWaypoints.add(new Waypoint(140,20,15,60));
        sWaypoints.add(new Waypoint(100,60,15,60));
        sWaypoints.add(new Waypoint(120,80,15,60));
        sWaypoints.add(new Waypoint(260,120,15,60));
        sWaypoints.add(new Waypoint(280,140,15,60));
        sWaypoints.add(new Waypoint(260,160,15,60));
        sWaypoints.add(new Waypoint(220,140,15,60));
        sWaypoints.add(new Waypoint(260,90,15,60));
        sWaypoints.add(new Waypoint(300,60,15,60));
        sWaypoints.add(new Waypoint(340,80,15,60));
        sWaypoints.add(new Waypoint(300,100,15,60));
        sWaypoints.add(new Waypoint(250,100,15,60));
        sWaypoints.add(new Waypoint(40,100,15,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}