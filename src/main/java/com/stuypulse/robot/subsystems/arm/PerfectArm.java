package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AFilter;
import com.stuypulse.stuylib.streams.angles.filters.ALowPassFilter;

import edu.wpi.first.math.geometry.Rotation2d;

public class PerfectArm extends Arm {

    private AFilter wristFilter = new ALowPassFilter(0.2).then(new ALowPassFilter(0.2).then(new ALowPassFilter(0.2)));
    private AFilter shoulderFilter = new ALowPassFilter(0.2).then(new ALowPassFilter(0.2).then(new ALowPassFilter(0.2)));

    @Override
    public Rotation2d getShoulderAngle() {
        return shoulderFilter.get(Angle.fromRotation2d(getShoulderTargetAngle())).getRotation2d();
    }

    @Override
    public Rotation2d getRelativeWristAngle() {
        return wristFilter.get(Angle.fromRotation2d(getWristTargetAngle().minus(getShoulderAngle()))).getRotation2d();
    }

	@Override
	protected void setShoulderVoltageImpl(double voltage) {}

	@Override
	protected void setWristVoltageImpl(double voltage) {}

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