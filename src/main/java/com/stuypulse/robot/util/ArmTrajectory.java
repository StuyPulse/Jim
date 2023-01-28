package com.stuypulse.robot.util;

import java.util.List;
import java.util.ArrayList;

public class ArmTrajectory {
    private final List<ArmState> states;

    public ArmTrajectory() {
        states = new ArrayList<>();
    }

    public List<ArmState> getStates() {
        return states;
    }

    public ArmTrajectory addState(double shoulderDegrees, double armDegrees) {
        states.add(new ArmState(shoulderDegrees, armDegrees));
        return this;
    }

    public ArmTrajectory addState(ArmTrajectory trajectory) {
        this.addState(trajectory.getStates().get(0).getShoulderRotation().getDegrees(), 
                        trajectory.getStates().get(0).getWristRotation().getDegrees());
        return this;
    }
} 