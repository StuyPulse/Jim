package com.stuypulse.robot.util;

public class AngleUtil {
    /**
     * Convert degree into the angle directly clockwise from referenceDegree
     */
    public static double degreeClockwiseFrom(double degree, double referenceDegree) {
        return 360 - (referenceDegree + degree);
    }
}
