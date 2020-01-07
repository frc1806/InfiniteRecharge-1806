package com.team1806.frc2019.auto.actions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class OutputTime implements Action {
	String splitName;
	public OutputTime(String splitName) {
		this.splitName = splitName;
	}
	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void update() {
		//Nothing!
	}

	@Override
	public void done() {
		System.out.println(splitName + " Time: " + Timer.getMatchTime());
	}

	@Override
	public void start() {
		//Nothing
	}

}
