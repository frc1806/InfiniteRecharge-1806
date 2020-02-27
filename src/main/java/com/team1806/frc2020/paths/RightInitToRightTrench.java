package com.team1806.frc2020.paths;

import com.team1806.lib.control.Path;

import java.util.ArrayList;

public class RightInitToRightTrench implements PathContainer{
    //Starts on the init line on the right and goes backwards through the trench picking up 5 Power Cells

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(131,250,0,0));
        sWaypoints.add(new PathBuilder.Waypoint(151,250,15,60));
        sWaypoints.add(new PathBuilder.Waypoint(220,295,70,60));
        sWaypoints.add(new PathBuilder.Waypoint(370,295,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }


    @Override
    public boolean isReversed() {
        return true;
    }
}
