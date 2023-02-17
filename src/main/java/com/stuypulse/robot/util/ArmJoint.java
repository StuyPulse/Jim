package com.stuypulse.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmJoint {
    public final DCMotor motor;
    public final double mass;
    public final double length;
    public final double moi;
    public final double radius;

    public ArmJoint(DCMotor motor, double mass, double length, double moi, double radius) {
        this.motor = motor;
        this.mass = mass;
        this.length = length;
        this.moi = moi;
        this.radius = radius;
    }
}
