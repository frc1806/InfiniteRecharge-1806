package com.team1806.lib.util;

import edu.wpi.first.wpilibj.Timer;
import org.junit.Assert;
import org.junit.Test;
import org.junit.Ignore;

public class MultiTriggerTest {
    @Ignore
    @Test
    public void test() {
        MultiTrigger trigger = new MultiTrigger(0.1);
        trigger.update(true);
        Assert.assertTrue(trigger.wasTapped());
        Assert.assertTrue(trigger.isPressed());

        trigger.update(true);
        Assert.assertFalse(trigger.wasTapped());
        Assert.assertTrue(trigger.isPressed());

        Timer.delay(0.05);
        Assert.assertFalse(trigger.isHeld());
        Assert.assertTrue(trigger.isPressed());

        Timer.delay(0.1);
        Assert.assertTrue(trigger.isHeld());
        Assert.assertTrue(trigger.isPressed());

        trigger.update(false);
        Assert.assertFalse(trigger.wasTapped());
        Assert.assertFalse(trigger.isPressed());
        Assert.assertFalse(trigger.isHeld());
    }
}
