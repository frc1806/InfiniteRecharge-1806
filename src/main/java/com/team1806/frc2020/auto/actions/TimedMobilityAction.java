package com.team1806.frc2020.auto.actions;

import com.team1806.frc2020.subsystems.Drive;
import com.team1806.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

public class TimedMobilityAction implements Action {
    double startTime;
    double runTime;
    Drive driveTrain = Drive.getInstance();

    public TimedMobilityAction(double _runTime) {
        startTime = Timer.getFPGATimestamp();
        runTime = _runTime;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();

        driveTrain.setHighGear(true);
    }

    @Override
    public void update() {
        driveTrain.setOpenLoop(new DriveSignal(.4,.4));
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= runTime;
    }

    @Override
    public void done() {
        driveTrain.stop();
    }
}
