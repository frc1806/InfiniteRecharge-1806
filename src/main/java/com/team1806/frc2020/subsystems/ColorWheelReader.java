package com.team1806.frc2020.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team1806.lib.util.ReflectingCSVWriter;

public class ColorWheelReader extends Subsystem {

    public enum ActualColor{
        Red, Blue, Green, Yellow;
    }
    public enum SensedColor {
        Red(0, 0, 0, 0, 0, 0, ActualColor.Blue),
        Blue(0, 0, 0, 0, 0, 0, ActualColor.Red),
        Green(0, 0, 0, 0, 0, 0, ActualColor.Yellow),
        Yellow(0, 0,0 , 0, 0, 0, ActualColor.Green);

        private double minRed;
        private double maxRed;

        private double minBlue;
        private double maxBlue;

        private double minGreen;
        private double maxGreen;

        private ActualColor mappedFieldColor;
        private SensedColor(double minRed, double maxRed, double minBlue, double maxBlue, double minGreen, double maxGreen, ActualColor MappedFieldColor){

        }
    }

    public enum ColorWheelControlState{
        BEING_TURNED, IDLE
    }
    private class PeriodicIO {
        //inputs
        double dRedValue;
        double dBlueValue;
        double dGreenValue;
        ActualColor lastColor;
        ActualColor currentColor;
        //outputs
        Color
    }

    private PeriodicIO mPeriodicIO;
    private ColorWheelControlState mColorWheelControlState;
    private int mSectionCount = 0;

    private static ColorWheelReader COLOR_WHEEL_READER = new ColorWheelReader();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;

    public static ColorWheelReader GetInstance(){
        return COLOR_WHEEL_READER;
    }

    private ColorWheelReader(){
        mColorWheelControlState = ColorWheelControlState.IDLE;
    }

    public void readPeriodicInputs() {
        mPeriodicIO.lastColor = mPeriodicIO.currentColor;
        mPeriodicIO.currentColor = //read from sensor and decide what actual color the sensor is on.
    }

    public void writePeriodicOutputs() {

    }



    public void startSensing(){
        mColorWheelControlState = ColorWheelControlState.BEING_TURNED;
    }
    public  void stopSensing(){
        mColorWheelControlState = ColorWheelControlState.IDLE;
    }


    public int getColorWheelRotationCount() {
        return mSectionCount/3.0d;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Color Wheel Control State", mPeriodicIO.ColorWheelControlState.toString());
        SmartDashboard.putString("Current Color", mPeriodicIO.currentColor.toString());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

    }

    public synchronized void startLogging(){
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/COLOR_WHEEL-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging(){
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }



}