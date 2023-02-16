package com.stuypulse.robot.util.AStar;

public class Ting {
    double shoulder;
    double wrist;

    public Ting(double shoulder, double wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    public double getShoulder() {
        return shoulder;
    }

    public double getWrist() {
        return wrist;
    }

    @Override
    public String toString() {
        return "[" + shoulder + ", " + wrist + "]";
    }
}
