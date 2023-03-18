package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.streams.filters.Derivative;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

public class PerfectArm extends Arm {

    private IFilter wristFilter = new LowPassFilter(0.2).then(new LowPassFilter(0.2).then(new LowPassFilter(0.2)));
    private IFilter shoulderFilter = new LowPassFilter(0.2).then(new LowPassFilter(0.2).then(new LowPassFilter(0.2)));

    private IFilter wristDerivative = new Derivative();
    private IFilter shoulderDerivative = new Derivative();

    public double getShoulderAngleRadians() {
        return shoulderFilter.get(getShoulderTargetAngleRadians());
    }

    public double getRelativeWristAngleRadians() {
        return wristFilter.get(getWristTargetAngleRadians() - getShoulderAngleRadians());
    }

	@Override
	protected void setShoulderVoltageImpl(double voltage) {}

	@Override
	protected void setWristVoltageImpl(double voltage) {}

    @Override
    public double getShoulderVelocityRadiansPerSecond() {
        return shoulderDerivative.get(getShoulderAngleRadians());
    }

    @Override
    public double getWristVelocityRadiansPerSecond() {
        return wristDerivative.get(getRelativeWristAngleRadians());
    }

}