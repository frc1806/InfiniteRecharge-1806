package com.team1806.lib.util.LED;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.List;

public interface LEDPattern {

    public Color getColorForPositionInString(int position);

    public void updateAnimation();
}
