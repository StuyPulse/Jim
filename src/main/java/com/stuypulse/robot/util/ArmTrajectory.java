package com.stuypulse.robot.util;

import java.util.List;
import java.util.ArrayList;

public class ArmTrajectory {
    public static ArmTrajectory fromStates(ArmState... states) {
        ArmTrajectory trajectory = new ArmTrajectory();
        for (ArmState state : states) {
            trajectory.addState(state);
        }
        return trajectory;
    }

    private final List<ArmState> states;

    public ArmTrajectory() {
        states = new ArrayList<>();
    }

    public List<ArmState> getStates() {
        return states;
    }

    public ArmTrajectory addState(ArmState state) {
        states.add(state);
        return this;
    }

    public ArmTrajectory addStates(List<ArmState> states) {
        for (ArmState state : states) {
            states.add(state);
        }
        return this;
    }

    // public ArmTrajectory addState(double shoulderDegrees, double wristDegrees) {
    //     return addState(new ArmState(shoulderDegrees, wristDegrees));
    // }

    public ArmTrajectory append(ArmTrajectory trajectory) {
        for (ArmState state: trajectory.getStates()) {
            states.add(state);
        }
        return this;
    }

    public ArmTrajectory flipped() {
        ArmTrajectory flipped = new ArmTrajectory();
        for (ArmState state : states) {
            flipped.addState(state.flip());
        }
        return flipped;
    }
} 