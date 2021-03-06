package com.team1806.lib.util.LED;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.List;

public interface LEDPattern {

    /**
     * Get the desired color for a given position in an LED String
     * @param position the desired position in the LED string, 0-indexed
     * @return the color the pattern needs in that position.
     */
    public Color getColorForPositionInString(int position);

    /**
     * Advances the animation one frame
     */
    public void updateAnimation();

    /**
     * Get the frame rate the animation is designed to be animated at.
     * @return the frame rate for the animation
     */
    public int getFPS();

}
