package frc.robot;

import static frc.robot.MathUtils.absClamp;

import org.junit.Assert;
import org.junit.Test;

public class MathUtilsTest {

    private void testAbsClampHelper(double expected, double value, double clamp) {
        Assert.assertEquals(expected, absClamp(value, clamp), 0);
    }

    @Test
    public void testAbsClamp() {
        testAbsClampHelper(0, 1, 0);
        testAbsClampHelper(0.25, 1, 0.25);
        testAbsClampHelper(-0.25, -1, 0.25);
    }

}