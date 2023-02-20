package com.stuypulse.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;

// TODO: add noise
public class TwoJointArmSimulation {

    private Vector<N4> state;
    private ArmDynamics dynamics;

    public TwoJointArmSimulation(Vector<N4> initialState, ArmDynamics dynamics) {
        state = initialState;
        this.dynamics = dynamics;
    }

    public TwoJointArmSimulation(double shoulderRadians, double wristRadians, double shoulderRadiansPerSecond, double wristRadiansPerSecond, ArmDynamics dynamics) {
        this(VecBuilder.fill(shoulderRadians, wristRadians, shoulderRadiansPerSecond, wristRadiansPerSecond), dynamics);
    } 

    public TwoJointArmSimulation(double shoulderRadians, double wristRadians, ArmDynamics dynamics) {
        this(shoulderRadians, wristRadians, 0, 0, dynamics);
    }

    public double getShoulderPositionRadians() {
        return state.get(0, 0);
    }

    public double getWristPositionRadians() {
        return state.get(1, 0);
    }

    public double getShoulderVelocityRadiansPerSecond() {
        return state.get(2, 0);
    }

    public double getWristVelocityRadiansPerSecond() {
        return state.get(3, 0);
    }

    public void update(Vector<N2> voltage, double dt) {
        state = dynamics.simulate(state, voltage, dt);
    }

    public void update(double shoulderVolts, double wristVolts, double dt) {
        update(VecBuilder.fill(shoulderVolts, wristVolts), dt);
    }

}
