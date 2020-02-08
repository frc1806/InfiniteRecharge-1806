package com.team1806.frc2020;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Set;

import com.team1806.frc2020.auto.modes.AutoModeBase;
import com.team1806.frc2020.auto.modes.DoNothingMode;

import com.team1806.frc2020.loops.ILooper;
import com.team1806.frc2020.loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.reflections.Reflections;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that allows a user to select which autonomous mode to execute from shuffleboard.
 */
public class AutoModeSelector {

    //public static final String AUTO_OPTIONS_DASHBOARD_KEY = "auto_options";
    public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "selected_auto_mode";
    public static final String AUTO_MODES_PACKAGE = "com.team1806.frc2020.auto.modes";
    private static SendableChooser<String> AUTO_CHOOSER = new SendableChooser<>();

    /**
     * Uses reflection to get every auto mode in the defined auto modes package. The idea being to remove a step from the process of adding an autonomous mode.
     */
    public static void initAutoModeSelector() {
        ArrayList<String> modesArray = new ArrayList<String>();
        Reflections reflections = new Reflections(AUTO_MODES_PACKAGE);
        Set<Class<? extends AutoModeBase>> modes = reflections.getSubTypesOf(AutoModeBase.class);
        for (Class<?> mode : modes) {
            AUTO_CHOOSER.addOption(mode.getSimpleName(), mode.getName());
            //modesArray.add(mode.getName()); Legacy Line for Chris's old dashboard
        }
        //String[] stringArray = new String[modesArray.size()];

        //SmartDashboard.putStringArray(AUTO_OPTIONS_DASHBOARD_KEY, modesArray.toArray(stringArray));
        //SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, mDefaultMode.mDashboardName);

    }


    /**
     * Checks the returned automode against every mode in the defined auto modes package. If there is a problem instantiating it, or the mode selected doesn't exist, will return a default auto.
     */
    public static AutoModeBase getSelectedAutoMode() {

        String selectedModeName = AUTO_CHOOSER.getSelected();
        Reflections reflections = new Reflections(AUTO_MODES_PACKAGE);
        Set<Class<? extends AutoModeBase>> modes = reflections.getSubTypesOf(AutoModeBase.class);

        for (Class<?> mode : modes) {
            if (selectedModeName.equals(mode.getName())) {
                try {
                    return (AutoModeBase) mode.getConstructor().newInstance();
                } catch (InstantiationException e) {
                    e.printStackTrace();
                    return fallBackToDefaultAuto(selectedModeName);
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                    return fallBackToDefaultAuto(selectedModeName);
                } catch (IllegalArgumentException e) {
                    e.printStackTrace();
                    return fallBackToDefaultAuto(selectedModeName);
                } catch (InvocationTargetException e) {
                    e.printStackTrace();
                    return fallBackToDefaultAuto(selectedModeName);
                } catch (NoSuchMethodException e) {
                    e.printStackTrace();
                    return fallBackToDefaultAuto(selectedModeName);
                } catch (SecurityException e) {
                    e.printStackTrace();
                    return fallBackToDefaultAuto(selectedModeName);
                } finally {
                }
            }
        }

        return fallBackToDefaultAuto(selectedModeName);
    }

    public static AutoModeBase fallBackToDefaultAuto(String wantedAutoMode) {
        DriverStation.reportError("Failed to select auto mode: " + wantedAutoMode, false);
        return new DoNothingMode();
    }

    public static String returnNameOfSelectedAuto() {
       /* return  SmartDashboard.getString(
                SELECTED_AUTO_MODE_DASHBOARD_KEY,
                "NO SELECTED MODE!!!!");

        */
        return AUTO_CHOOSER.getSelected();
    }

    public static void registerDisabledLoop(ILooper in) {
        in.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, AUTO_CHOOSER.getSelected());
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }
}