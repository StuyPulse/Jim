package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class PerfectArm extends Arm {

    @Override
    public Rotation2d getShoulderAngle() {
        return getShoulderTargetAngle();
    }

    @Override
    public Rotation2d getRelativeWristAngle() {
        return getWristTargetAngle().minus(getShoulderAngle());
    }

	@Override
	protected void setShoulderVoltageImpl(double voltage) {}

	@Override
	protected void setWristVoltageImpl(double voltage) {}

}