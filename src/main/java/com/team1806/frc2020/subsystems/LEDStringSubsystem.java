package com.team1806.frc2020.subsystems;

import com.team1806.frc2020.controlboard.ControlBoard;
import com.team1806.frc2020.loops.ILooper;
import com.team1806.frc2020.loops.Loop;
import com.team1806.lib.util.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStringSubsystem extends Subsystem {


    private AddressableLED mLED;
    private AddressableLEDBuffer mLEDBuffer;
    private int mNumOfLEDs;

    private LEDPattern currentPattern;


    public LEDStringSubsystem(int port, int mNumOfLEDs) {
        this.mLED = new AddressableLED(port);
        this.mNumOfLEDs = mNumOfLEDs;
        this.mLEDBuffer = new AddressableLEDBuffer(mNumOfLEDs);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.setData(mLEDBuffer);
        currentPattern = LEDPattern.BLACK_AND_WHITE;
        StartLED();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (LEDStringSubsystem.this) {
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LEDStringSubsystem.this) {
                    currentPattern.updateAnimation();
                    for(int i =0; i < mLEDBuffer.getLength(); i++){
                        mLEDBuffer.setLED(i, currentPattern.getColorForPositionInString(i));
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
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
