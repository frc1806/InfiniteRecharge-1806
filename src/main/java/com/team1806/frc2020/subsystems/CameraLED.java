package com.team1806.frc2020.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class CameraLED extends Subsystem {


    public AddressableLED mLED;
    public AddressableLEDBuffer mLEDBuffer;
    public int mNumOfLEDs;
    public DriverStation mDriverStation;
    public Timer timer;
    private PeriodicIO mPeriodicIO;


    enum CameraLED_ControlState {

        kVision,
    }

    public CameraLED(int port, int mNumOfLEDs){
        this.mLED = new AddressableLED(port);
        this.mNumOfLEDs = mNumOfLEDs;
        this.mDriverStation = DriverStation.getInstance();
        this.mLEDBuffer = new AddressableLEDBuffer(mNumOfLEDs);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.setData(mLEDBuffer);
        this.timer = new Timer();

        setControlState(CameraLED_ControlState.kVision);

    }

    public void readPeriodicInputs(){ }

    public void writePeriodicOutputs(){
        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            mLEDBuffer.setRGB(i, 0, 255, 0);
        }

        mLED.setData(mLEDBuffer);

    }

    private class PeriodicIO {
        CameraLED_ControlState mCameraLED_ControlState;

    }

    private void setControlState(CameraLED_ControlState state) {
        mPeriodicIO.mCameraLED_ControlState = state;

    }

    public void StartLED(){
        mLED.start();
    }

    public void StopLED() {
        mLED.stop();
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

}
