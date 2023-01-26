package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.control.Controller;

public class FeedforwardArm extends Controller {
    private final Number kG;

    public FeedforwardArm(Number kG) {
        this.kG = kG;
    }

    protected double calculate(double setpoint, double measurement) {
        // VGravity at any angle is equal to the cosine of the angle from the horizontal * VGravity at the horizontal 
        
        // TODO: FLIPPING
        return this.kG.doubleValue() * Math.cos(setpoint);
    }
}