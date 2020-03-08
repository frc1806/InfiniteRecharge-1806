package com.team1806.lib.util;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.List;

public class LEDPattern {

    public static final LEDPattern VISION_GREEN = new LEDPattern(Arrays.asList(Color.kGreen), 0);
    public static final LEDPattern BLACK_AND_WHITE = new LEDPattern(Arrays.asList(Color.kBlack, Color.kBlack, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite), 1);
    public static final LEDPattern ECTOPLASM = new LEDPattern(Arrays.asList(Color.kDarkGreen, Color.kLimeGreen, Color.kLimeGreen, Color.kDarkGreen, Color.kBlack, Color.kBlack, Color.kBlack, Color.kBlack), 2);

    private List<Color> pattern;
    private int animationStepAmount;
    private int currentAnimationStep;

    public LEDPattern(List<Color> pattern, int animationStepAmount) {
        this.pattern = pattern;
        this.animationStepAmount = animationStepAmount;
    }

    public Color getColorForPositionInString(int position){
        return pattern.get((position-currentAnimationStep) % pattern.size());
    }

    public void updateAnimation(){
        currentAnimationStep += animationStepAmount;
    }
}
