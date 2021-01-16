package com.team1806.frc2020.subsystems;

import com.team1806.frc2020.loops.ILooper;
import com.team1806.frc2020.loops.Loop;
import com.team1806.lib.util.LED.LEDPattern;
import com.team1806.lib.util.LED.ScrollingLEDPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.util.Color;
import java.util.stream.IntStream;

public class LEDStringSubsystem extends Subsystem {


    private AddressableLED mLED;
    private AddressableLEDBuffer mLEDBuffer;
    private int mNumOfLEDs;
    private boolean stopOnDisable;
    private double lastUpdateTime;

    private LEDPattern currentPattern;


    public LEDStringSubsystem(int port, int mNumOfLEDs, boolean stopOnDisable) {
        lastUpdateTime = 0;
        this.mLED = new AddressableLED(port);
        this.mNumOfLEDs = mNumOfLEDs;
        this.mLEDBuffer = new AddressableLEDBuffer(mNumOfLEDs);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.setData(mLEDBuffer);
        currentPattern = ScrollingLEDPattern.BLACK_AND_WHITE;
        StartLED();
        this.stopOnDisable = stopOnDisable;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                StartLED();
            }

            @Override
            public void onLoop(double timestamp) {
                if(currentPattern.getFPS() == 0)
                {
                    return;
                }
                if(timestamp - lastUpdateTime > 1.0 / currentPattern.getFPS()) {
                    currentPattern.updateAnimation();
                    IntStream.range(0, mLEDBuffer.getLength()).forEach(i -> mLEDBuffer.setLED(i, currentPattern.getColorForPositionInString(i)));
                    mLED.setData(mLEDBuffer);
                    lastUpdateTime = timestamp;
                }
            }

            @Override
            public void onStop(double timestamp) {
                if(stopOnDisable)
                {
                    IntStream.range(0, mLEDBuffer.getLength()).forEach(i -> mLEDBuffer.setLED(i, Color.kBlack));
                    mLED.setData(mLEDBuffer);
                }
                StopLED();
            }
        });
    }



    public void StartLED() {
        mLED.start();
    }

    public void StopLED() {
        mLED.stop();
    }

    @Override
    public void stop() {
        mLED.stop();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    public void setPattern(LEDPattern currentPattern) {
        this.currentPattern = currentPattern;
    }

}
