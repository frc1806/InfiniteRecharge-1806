package com.team1806.frc2020.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.team1806.frc2020.Constants;
import com.team1806.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorWheelReader extends Subsystem {

    private static ColorWheelReader COLOR_WHEEL_READER = new ColorWheelReader();
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 mColorSensor;
    private final ColorMatch mColorMatcher;
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    private PeriodicIO mPeriodicIO;
    private ColorWheelControlState mColorWheelControlState;
    private int mSectionCount = 0;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
    private ColorWheelReader() {

        mPeriodicIO = new PeriodicIO();
        mColorSensor = new ColorSensorV3(i2cPort);
        mColorMatcher = new ColorMatch();
        mPeriodicIO.counterEnabled = false;
        mColorWheelControlState = ColorWheelControlState.IDLE;
        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);
        mColorMatcher.setConfidenceThreshold(0);
        mPeriodicIO.lastColor = MatchedColor.kUnknown;
        mPeriodicIO.currentColor = MatchedColor.kUnknown;
        mPeriodicIO.detectedColor = Color.kLimeGreen;
        mPeriodicIO.matchResult = new ColorMatchResult(Color.kLimeGreen, 100); //Hi 1678
    }

    public static ColorWheelReader GetInstance() {
        return COLOR_WHEEL_READER;
    }

    private MatchedColor matcher(ColorMatchResult matchResult) {
        if (matchResult.confidence > Constants.kColorMatcherRequriedConfidance) {
            if (matchResult.color == kRedTarget) {
                return MatchedColor.kBlue;
            } else if (matchResult.color == kGreenTarget) {
                return MatchedColor.kYellow;
            } else if (matchResult.color == kBlueTarget) {
                return MatchedColor.kRed;
            } else if (matchResult.color == kYellowTarget) {
                return MatchedColor.kGreen;
            } else {
                return MatchedColor.kUnknown;
            }
        } else {
            return MatchedColor.kUnknown;
        }

    }

    public void readPeriodicInputs() {
        mPeriodicIO.lastColor = mPeriodicIO.currentColor;
        mPeriodicIO.detectedColor = mColorSensor.getColor();
        mPeriodicIO.matchResult = mColorMatcher.matchClosestColor(mPeriodicIO.detectedColor);
        MatchedColor matchedColor = matcher(mPeriodicIO.matchResult);
        mPeriodicIO.currentColor = matchedColor == MatchedColor.kUnknown ? mPeriodicIO.currentColor : matchedColor;
        if (mPeriodicIO.counterEnabled && mPeriodicIO.currentColor != mPeriodicIO.lastColor) {
            mSectionCount++;
        }
    }

    public void writePeriodicOutputs() {

    }

    public void startSensing() {
        mColorWheelControlState = ColorWheelControlState.BEING_TURNED;
        mPeriodicIO.counterEnabled = true;

    }

    public void stopSensing() {
        mColorWheelControlState = ColorWheelControlState.IDLE;
        mPeriodicIO.counterEnabled = false;
        mSectionCount = 0;
    }

    public double getColorWheelRotationCount() {
        return mSectionCount / 8.0d;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Color Wheel Control State", mColorWheelControlState.toString());
        SmartDashboard.putString("Color Sensor Current Color", mPeriodicIO.currentColor.toString());
        SmartDashboard.putNumber("Color Sensor Red", mPeriodicIO.detectedColor.red);
        SmartDashboard.putNumber("Color Sensor Green", mPeriodicIO.detectedColor.green);
        SmartDashboard.putNumber("Color Sensor Blue", mPeriodicIO.detectedColor.blue);
        SmartDashboard.putNumber("Color Sensor Confidence", mPeriodicIO.matchResult.confidence);


        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/COLOR_WHEEL-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public MatchedColor getMatchedColor() {
        return mPeriodicIO.currentColor;
    }

    public void stop() {


    }

    public boolean checkSystem() {
        return true;

    }

    public enum MatchedColor {
        kRed, kGreen, kBlue, kYellow, kUnknown
    }

    public enum ColorWheelControlState {
        BEING_TURNED, IDLE
    }

    private class PeriodicIO {
        //inputs
        public boolean counterEnabled;
        public MatchedColor lastColor;
        public Color detectedColor;
        public ColorMatchResult matchResult;
        public MatchedColor currentColor; //This is the sensor color!!

    }

}