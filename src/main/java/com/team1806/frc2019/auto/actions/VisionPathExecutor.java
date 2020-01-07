package com.team1806.frc2019.auto.actions;

import com.team1806.frc2019.auto.actions.Action;
import com.team1806.frc2019.auto.modes.VisionMode;

public class VisionPathExecutor implements Action {
    VisionMode visionMode;
    boolean started = false;
    @Override
    public boolean isFinished() {
        return started && visionMode.getIsDone();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        visionMode.stop();
    }

    @Override
    public void start() {
        visionMode = new VisionMode();
        visionMode.run();
        started = true;
    }
}
