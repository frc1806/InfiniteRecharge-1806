package com.team1806.lib.util.LED;

public class LEDPatternSegment {
    int length;
    LEDPattern pattern;

    public LEDPatternSegment(int length, LEDPattern pattern) {
        this.length = length;
        this.pattern = pattern;
    }

    public int getLength() {
        return length;
    }

    public LEDPattern getPattern() {
        return pattern;
    }
}
