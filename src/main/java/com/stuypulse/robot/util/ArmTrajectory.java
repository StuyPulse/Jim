package com.stuypulse.robot.util;

import java.util.List;

import java.util.ArrayList;

public class ArmTrajectory {

    private final List<ArmState> states;

    public ArmTrajectory() {
        states = new ArrayList<>();
    }
    
    public ArmTrajectory addState(Number shoulderDegrees, Number wristDegrees) {
        return addState(new ArmState(shoulderDegrees, wristDegrees));
    }

    public ArmTrajectory addState(ArmState armState) {
        states.add(armState);
        return this;
    }

    public List<ArmState> getStates() {
        return states;
    }

    public int getEntries() {
        return states.size();
    }

}
