package com.team1806.lib.util.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Random;
import java.util.function.IntPredicate;
import java.util.stream.IntStream;

/**
 * This class starts with a given pattern and corrupts it every 100 loops, and adds a 1 frame corruption if conditions are met.
 */
public class GlitchyLEDPattern implements LEDPattern {

    private static final int INTS_FOR_FUN_SIZE = 16;
    private static final int RANDOM_GLITCH_EFFECT_CHOCIES = 14;

    private Random mRandom;
    private int mAnimationCounter;
    private AddressableLEDBuffer mLEDBuffer;
    private IntStream mIntsForFun;

    public GlitchyLEDPattern(LEDPattern startingPoint, int size){
        mRandom = new Random(System.currentTimeMillis());
        mLEDBuffer = new AddressableLEDBuffer(size);
        //Init an internal buffer to the current state of what the starting point would push to the string.
        for(int i = 0; i < mLEDBuffer.getLength(); i++){
            mLEDBuffer.setLED(i, startingPoint.getColorForPositionInString(i));
        }
        mAnimationCounter = 0;
        mIntsForFun = mRandom.ints(INTS_FOR_FUN_SIZE);
    }


    @Override
    public Color getColorForPositionInString(int position) {
        if(mIntsForFun.noneMatch(new IntPredicate() {
            @Override
            public boolean test(int i) {
                return i == mAnimationCounter % INTS_FOR_FUN_SIZE;
            }
        })) {
            return mLEDBuffer.getLED(position);
        }
        else{
            return getGlitchEffectedColor(position);
        }

    }

    @Override
    public void updateAnimation() {
        //choose your corruption
        mIntsForFun = mRandom.ints(INTS_FOR_FUN_SIZE);
        mAnimationCounter++;
        if(mAnimationCounter % 100 == 0 ){
            switch(mRandom.nextInt() % 10){
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                    break; // do nothing. That's a glitchy thing, right?
                case 6:
                    //replace ~20% of the LEDs with random colors
                    for(int i = 0; i < mLEDBuffer.getLength() * .2; i++){
                        mLEDBuffer.setLED(mRandom.nextInt() % mLEDBuffer.getLength(), generateRandomColor());
                    }
                    break;
                case 7:
                    //replace ~20% of the LEDs with glitch colors
                    for(int i = 0; i < mLEDBuffer.getLength() * .2; i++){
                        int afflictedPosition = mRandom.nextInt() % mLEDBuffer.getLength();
                        mLEDBuffer.setLED(afflictedPosition , getGlitchEffectedColor(afflictedPosition));
                    }
                    break;
                case 8:
                    //Random shift
                    for(int i = 0; i < mLEDBuffer.getLength(); i++){
                        mLEDBuffer.setLED(i, mLEDBuffer.getLED((i + mIntsForFun.toArray()[3]) % mLEDBuffer.getLength()));
                    }
                    break;
                case 9:
                    //flip buffer
                    for(int i = 0; i < mLEDBuffer.getLength(); i++){
                        mLEDBuffer.setLED(i, mLEDBuffer.getLED((mLEDBuffer.getLength()-1) - i));
                    }
                case 10:
                    //apply random color to random first percent of the buffer.
                    for(int i = 0; i < mLEDBuffer.getLength() * mRandom.nextDouble(); i++){
                        mLEDBuffer.setLED(i, generateRandomColor());
                    }
                    break;
                case 11:
                    //replace random percent of the buffer with glitches from front.
                    for(int i = 0; i < mLEDBuffer.getLength() * mRandom.nextDouble(); i++){
                        mLEDBuffer.setLED(i , getGlitchEffectedColor(i));
                    }
                    break;
                case 12:
                    //apply random color to random last percent of the buffer.
                    for(int i = mLEDBuffer.getLength(); i > mLEDBuffer.getLength() * mRandom.nextDouble(); i--){
                        mLEDBuffer.setLED(i, generateRandomColor());
                    }
                    break;
                case 13:
                    //replace random percent of the buffer with glitches from back.
                    for(int i = mLEDBuffer.getLength(); i < mLEDBuffer.getLength() * mRandom.nextDouble(); i--){
                        mLEDBuffer.setLED(i , getGlitchEffectedColor(i));
                    }
                    break;
            }
        }
    }

    private Color generateRandomColor(){
        return new Color(mRandom.nextDouble(), mRandom.nextDouble(), mRandom.nextDouble());
    }

    private Color getGlitchEffectedColor(int position){
        Color originalColor = mLEDBuffer.getLED(position);
        switch(mIntsForFun.toArray()[1] % RANDOM_GLITCH_EFFECT_CHOCIES){
            default:
            case 0:
                return Color.kLimeGreen; //green
            case 1:
                return Color.kBlack; //black
            case 2:
                return Color.kRed; //red
            case 3:
                return Color.kAquamarine; // A common glitch color in old NES games, when SMB3 glitches out, the screen goes aquamarine a lot.
            case 4:
                return Color.kWhite; //rwhite
            case 5:
                //new random color
                return generateRandomColor();
            case 6:
                //invert color
                return new Color (1.0-originalColor.red, 1.0-originalColor.green, 1.0-originalColor.blue);
            case 7:
                //flip red and green
                return new Color (originalColor.green, originalColor.red, originalColor.blue);
            case 8:
                //flip red and blue
                return  new Color(originalColor.blue, originalColor.green, originalColor.red);
            case 9:
                //flip blue and green
                return new Color(originalColor.red, originalColor.blue, originalColor.green);
            case 10:
                //left shift color values
                return new Color(originalColor.green, originalColor.blue, originalColor.red);
            case 11:
                //grab a random pixel out of the saved buffer
                return mLEDBuffer.getLED( mRandom.nextInt() % mLEDBuffer.getLength());
            case 12:
                //shift a random amount in the buffer
                return mLEDBuffer.getLED((position + mIntsForFun.toArray()[2]) % mLEDBuffer.getLength());
            case 13:
                return Color.kDarkGray;
        }
    }
}
