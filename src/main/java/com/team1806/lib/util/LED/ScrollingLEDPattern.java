package com.team1806.lib.util.LED;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.List;

public class ScrollingLEDPattern implements LEDPattern {
    public static final ScrollingLEDPattern VISION_GREEN = new ScrollingLEDPattern(Arrays.asList(Color.kGreen), 0);
    public static final ScrollingLEDPattern BLACK_AND_WHITE = new ScrollingLEDPattern(Arrays.asList(Color.kBlack, Color.kBlack, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite), 1);
    public static final ScrollingLEDPattern ECTOPLASM = new ScrollingLEDPattern(Arrays.asList(Color.kDarkGreen, Color.kLimeGreen, Color.kLimeGreen, Color.kDarkGreen, Color.kBlack, Color.kBlack, Color.kBlack, Color.kBlack), 2);

    private List<Color> pattern;
    private int animationStepAmount;
    private int currentAnimationStep;

    public ScrollingLEDPattern(List<Color> pattern, int animationStepAmount) {
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
