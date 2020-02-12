package com.team1806.frc2020.subsystems;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team1806.lib.util.ReflectingCSVWriter;

public class ColorWheelReader extends Subsystem {

    public enum ActualColor{
        Red, Blue, Green, Yellow,
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
/*
    private enum MatchedColor{

        kRed, kGreen, kBlue, kYellow, kUnknown

    }
*/
    public enum ColorWheelControlState{
        BEING_TURNED, IDLE
    }
    private class PeriodicIO {
        //inputs
        double dRedValue;
        double dBlueValue;
        double dGreenValue;
        //MatchedColor lastColor;
        //Color detectedColor;
        //ColorMatchResult matchResult;
        //MatchedColor currentColor;

        public ColorWheelControlState ColorWheelState;

    }

    private PeriodicIO mPeriodicIO;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    //private final ColorSensorV3 mColorSensor = new ColorSensorV3(i2cPort);
    //private final ColorMatch mColorMatcher = new ColorMatch();
    private ColorWheelControlState mColorWheelControlState;
    private int mSectionCount = 0;

    //private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    //private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    //private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    //private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private static ColorWheelReader COLOR_WHEEL_READER = new ColorWheelReader();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;

    public static ColorWheelReader GetInstance(){
        return COLOR_WHEEL_READER;
    }

    private ColorWheelReader(){
        mColorWheelControlState = ColorWheelControlState.IDLE;
        //mColorMatcher.addColorMatch(kBlueTarget);
        //mColorMatcher.addColorMatch(kGreenTarget);
        //mColorMatcher.addColorMatch(kRedTarget);
        //mColorMatcher.addColorMatch(kYellowTarget);
    }
/*
    private MatchedColor matcher(ColorMatchResult matchResult) {
        if (matchResult == kRedTarget) {
            return kRed;
        } else if (matchResult == kGreenTarget) {
            return kGreen;
        } else if (matchResult == kBlueTarget) {
            return kBlue;
        } else if (matchResult == kYellow) {
            return kYellow;
        } else {
            return kUnknown;
        }

    }
*/

    public void readPeriodicInputs() {
        //mPeriodicIO.lastColor = mPeriodicIO.currentColor;
        //mPeriodicIO.detectedColor = mColorSensor.getColor();
        //mPeriodicIO.matchResult = mColorMatcher.matchClosestColor(detectedColor);
        //mPeriodicIO.currentColor = matcher(matchResult);

    }

    public void writePeriodicOutputs() {

    }



    public void startSensing(){
        mColorWheelControlState = ColorWheelControlState.BEING_TURNED;
    }
    public  void stopSensing(){
        mColorWheelControlState = ColorWheelControlState.IDLE;
    }


    public double getColorWheelRotationCount() {
        return mSectionCount/3.0d;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Color Wheel Control State", mPeriodicIO.ColorWheelState.toString());
        //SmartDashboard.putString("Current Color", mPeriodicIO.currentColor.toString());
        //SmartDashboard.putNumber("Red", detectedColor.red);
        //SmartDashboard.putNumber("Green", detectedColor.green);
        //SmartDashboard.putNumber("Blue", detectedColor.blue);
        //SmartDashboard.putNumber("Confidence", matchResult.confidence);


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


    public void stop(){



    }

    public boolean checkSystem(){
        return true;

    }

}