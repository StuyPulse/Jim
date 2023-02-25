package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;

import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.TwoJointArmSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimArm extends Arm {

    private final ArmDynamics dynamics;
    private final TwoJointArmSimulation simulation;

    private double shoulderVolts;
    private double wristVolts;

    public SimArm() { 
        dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);
        simulation = new TwoJointArmSimulation(-Math.PI/2, Math.PI/2, dynamics);
        
        shoulderVolts = 0;
        wristVolts = 0;
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRadians(simulation.getShoulderPositionRadians());
    }

    @Override
    public Rotation2d getRelativeWristAngle() {
        return Rotation2d.fromRadians(simulation.getWristPositionRadians());
    }

	@Override
	protected void setShoulderVoltage(double voltage) {
		shoulderVolts = voltage;
	}

	@Override
	protected void setWristVoltage(double voltage) {
		wristVolts = voltage;
	}

    @Override
    public void periodicallyCalled() {
        // if (!isWristFeedbackEnabled()) {
        //     simulation.update(0, wristVolts, Settings.DT);
        // } else {
            simulation.update(shoulderVolts, wristVolts, Settings.DT);
        // // }
    }
}