package com.stuypulse.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

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

    public ArmTrajectory wristMovesUpFirst(ArmState initialState) {
        ArmTrajectory trajectory = new ArmTrajectory();

        var up = Rotation2d.fromDegrees(+90);

        trajectory.addState(new ArmState(
            initialState.getShoulderState(),
            up
        ));

        trajectory.addState(new ArmState(
            states.get(0).getShoulderState(),
            up
        ));

        for (int i = 0; i < states.size() - 1; i++) {
            trajectory.addState(states.get(i));

            trajectory.addState(new ArmState(
                states.get(i).getShoulderState(),
                up
            ));

            trajectory.addState(new ArmState(
                states.get(i+1).getShoulderState(),
                up
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
