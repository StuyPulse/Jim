package com.stuypulse.robot.util;

import java.util.List;
import java.util.Optional;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.util.InterpolatingTreeMap;

import java.util.ArrayList;

public class ArmTrajectory {
    public static ArmTrajectory fromStates(ArmState... states) {
        ArmTrajectory trajectory = new ArmTrajectory();
        for (ArmState state : states) {
            trajectory.addState(state);
        }
        return trajectory;
    }

    private double time;
    private final TimeInterpolatableBuffer<ArmState> trajectory;
    private final List<ArmState> trajectoryList;

    public ArmTrajectory() {
        time = 0;
        trajectory = TimeInterpolatableBuffer.createBuffer(Double.MAX_VALUE);
        trajectoryList = new ArrayList<>();
    }

    public Optional<ArmState> calculate(double time) { 
        return trajectory.getSample(time);
    }

    public ArmTrajectory addState(ArmState state) {
        trajectory.addSample(time, state);
        trajectoryList.add(state);

        time += 1.0 / Settings.Arm.MAX_DEGREES_PER_SECOND;
        return this;
    }

    public double getTotalTime() {
        return time;
    }

    public boolean isDone(double time) {
        return trajectoryList.isEmpty() || (time < 0 || getTotalTime() < time);
    }

    public ArmTrajectory addStates(List<ArmState> states) {
        for (ArmState state : states) {
            addState(state);
        }
        return this;
    }

    // public ArmTrajectory addState(double shoulderDegrees, double wristDegrees) {
    //     return addState(new ArmState(shoulderDegrees, wristDegrees));
    // }

    public ArmTrajectory append(ArmTrajectory other) {
        addStates(other.trajectoryList);
        return this;
    }

    public ArmTrajectory flipped() {
        ArmTrajectory flipped = new ArmTrajectory();
        for (ArmState state : trajectoryList) {
            flipped.addState(state.flip());
        }
        return flipped;
    }
} 