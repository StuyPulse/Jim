/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.arm;

import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.TwoJointArmSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimArm extends Arm {

    private final ArmDynamics dynamics;
    private final TwoJointArmSimulation simulation;

    private double shoulderVolts;
    private double wristVolts;

    protected SimArm() {
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
	protected void setShoulderVoltageImpl(double voltage) {
		shoulderVolts = voltage;
	}

	@Override
    protected void setWristVoltageImpl(double voltage) {
		wristVolts = voltage;
	}

    @Override
    public void periodicallyCalled() {
        simulation.update(shoulderVolts, wristVolts, Settings.DT);
    }

    @Override
    public double getShoulderVelocityRadiansPerSecond() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getWristVelocityRadiansPerSecond() {
        // TODO Auto-generated method stub
        return 0;
    }
}
