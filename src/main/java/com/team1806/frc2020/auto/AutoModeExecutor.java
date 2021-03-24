package com.team1806.frc2020.auto;

import java.util.concurrent.atomic.AtomicBoolean;

import com.team1806.frc2020.auto.modes.AutoModeBase;
import com.team1806.lib.util.CrashTrackingRunnable;

/**
 * This class selects, runs, and (if necessary) stops a specified autonomous mode.
 */
public class AutoModeExecutor {
    private static AutoModeExecutor mInstance = null;
    public static AtomicBoolean autoRunning = new AtomicBoolean(false);
    private AutoModeBase mAutoMode = null;
    private Thread mThread = null;

    public AutoModeExecutor() {}

    public static AutoModeExecutor getInstance() {
        if (mInstance == null) {
            mInstance = new AutoModeExecutor();
        }

        return mInstance;
    }

    public void start() {
        if (mThread != null) {
            mThread.start();
        }
        autoRunning.set(true);
    }

    public boolean isStarted() {
        return mAutoMode != null && mAutoMode.isActive() && mThread != null && mThread.isAlive();
    }

    public void reset() {
        if (isStarted()) {
            stop();
        }
        autoRunning.set(false);
        mAutoMode = null;
    }

    public void stop() {
        if (mAutoMode != null) {
            mAutoMode.stop();
        }
        autoRunning.set(false);
        //mThread = null;
    }

    public AutoModeBase getAutoMode() {
        return mAutoMode;
    }

    public void setAutoMode(AutoModeBase new_auto_mode) {
        mAutoMode = new_auto_mode;
        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                if (mAutoMode != null) {
                    mAutoMode.run();
                }
            }
        });
    }

    public boolean isInterrupted() {
        if (mAutoMode == null) {
            return false;
        }
        return mAutoMode.getIsInterrupted();
    }

    public void interrupt() {
        if (mAutoMode == null) {
            return;
        }
        autoRunning.set(false);
        mAutoMode.interrupt();
    }

    public void resume() {
        if (mAutoMode == null) {
            return;
        }
        autoRunning.set(true);
        mAutoMode.resume();
    }
}
