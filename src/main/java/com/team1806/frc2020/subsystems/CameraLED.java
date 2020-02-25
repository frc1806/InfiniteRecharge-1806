package com.team1806.frc2020.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
public class CameraLED extends Subsystem {


    public AddressableLED mLED;
    public AddressableLEDBuffer mLEDBuffer;
    public int mNumOfLEDs;
    private PeriodicIO mPeriodicIO;


    enum CameraLED_ControlState {

        kVision,
    }

    public CameraLED(int port, int mNumOfLEDs){
        this.mLED = new AddressableLED(port);
        this.mNumOfLEDs = mNumOfLEDs;
        this.mLEDBuffer = new AddressableLEDBuffer(mNumOfLEDs);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.setData(mLEDBuffer);
        mPeriodicIO = new PeriodicIO();
        mPeriodicIO.cameraLED_ControlState = CameraLED_ControlState.kVision;

    }

    public void readPeriodicInputs(){ }

    public void writePeriodicOutputs(){
        for (int i = 0; i < mLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the rgb values for green
            mLEDBuffer.setRGB(i, 0, 255, 0);
        }

        mLED.setData(mLEDBuffer);

    }

    private class PeriodicIO {
        CameraLED_ControlState cameraLED_ControlState;

    }

    private void setControlState(CameraLED_ControlState state) {
        mPeriodicIO.cameraLED_ControlState = state;

    }

    public void StartLED(){
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

}
