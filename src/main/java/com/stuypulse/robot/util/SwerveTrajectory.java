package com.stuypulse.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;

public class SwerveTrajectory {

    private final List<SwerveState> states;

    public SwerveTrajectory() {
        states = new ArrayList<>();
    }
    
    public SwerveTrajectory addState(Pose2d pose) {
        states.add(new SwerveState(pose));
        return this;
    }

    public SwerveTrajectory addState(Number X, Number Y, Number omega) {
        states.add(new SwerveState(X, Y, omega));
        return this;
    }

    public List<SwerveState> getStates() {
        return states;
    }

    public int getEntries() {
        return states.size();
    }

}
