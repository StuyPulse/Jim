package com.stuypulse.robot.util;

public class ArmJoint {
    public final double mass;
    public final double length;
    public final double moi;
    public final double radius;

    public ArmJoint(double mass, double length, double moi, double radius) {
        this.mass = mass;
        this.length = length;
        this.moi = moi;
        this.radius = radius;
    }
}
