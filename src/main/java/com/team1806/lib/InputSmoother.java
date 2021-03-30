package com.team1806.lib;

public class InputSmoother {
    public static double smoothInput(double input)
    {
        return Math.pow(input, 3) / Math.sqrt(Math.abs(input));
    }
}
