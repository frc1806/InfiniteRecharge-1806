package com.team1806.lib.util.LED;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.List;

public class ScrollingLEDPattern implements LEDPattern {
    public static final ScrollingLEDPattern VISION_GREEN = new ScrollingLEDPattern(Arrays.asList(Color.kGreen), 0, 0);
    public static final ScrollingLEDPattern BLACK_AND_WHITE = new ScrollingLEDPattern(Arrays.asList(Color.kBlack, Color.kBlack, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite), 1, 60);
    public static final ScrollingLEDPattern ECTOPLASM = new ScrollingLEDPattern(Arrays.asList(Color.kDarkGreen, Color.kLimeGreen, Color.kLimeGreen, Color.kDarkGreen, Color.kBlack, Color.kBlack, Color.kBlack, Color.kBlack), 2, 60);

    private List<Color> pattern;
    private int animationStepAmount;
    private int currentAnimationStep;
    private int FPS;

    public ScrollingLEDPattern(List<Color> pattern, int animationStepAmount, int FPS) {
        this.pattern = pattern;
        this.animationStepAmount = animationStepAmount;
        this.FPS = FPS;
    }

    /**
     *{@inheritDoc}
     */
    public Color getColorForPositionInString(int position){
        return pattern.get(Math.abs(position-currentAnimationStep) % pattern.size());
    }

    /**
     *{@inheritDoc}
     */
    public void updateAnimation(){
        currentAnimationStep += animationStepAmount;
    }

    /**
     *{@inheritDoc}
     */
    public int getFPS(){
        return FPS;
    }
}
