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
        for (ArmState state: trajectory.getStates()) {
            states.add(state);
        }
        return this;
    }

    public void updateState(int index, ArmState newState) {
        this.getStates().set(index, newState);
    }

    public ArmTrajectory switchSides() {
        for (int i = 0; i < states.size(); i++) {
            updateState(i, new ArmState(
                            -180 - states.get(i).getShoulderRotation().getDegrees(), 
                            -180 - states.get(i).getWristRotation().getDegrees()));
        }
        return this;
    }
} 