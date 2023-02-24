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

    public ArmTrajectory wristMovesFirst() {
        ArmTrajectory trajectory = new ArmTrajectory();

        for (int i = 0; i < states.size() - 1; i++) {
            trajectory.addState(states.get(i));

            trajectory.addState(new ArmState(
                states.get(i).getShoulderState(),
                states.get(i+1).getWristState()
            ));
        }

        trajectory.addState(states.get(states.size() - 1));

        return trajectory;
    }

    public List<ArmState> getStates() {
        return states;
    }

    public int getEntries() {
        return states.size();
    }

}
