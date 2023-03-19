package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;

import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.TwoJointArmSimulation;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimArm extends Arm {

    private final ArmDynamics dynamics;
    private final TwoJointArmSimulation simulation;

    private SmartNumber shoulderVolts = new SmartNumber("Arm/Shoulder/Sim Voltage (Test)", 0);
    private SmartNumber wristVolts = new SmartNumber("Arm/Wrist/Sim Voltage (Test)", 0);

    protected SimArm() { 
        dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);
        simulation = new TwoJointArmSimulation(-Math.PI/2, Math.PI/2, dynamics);
        
        // shoulderVolts = 0;
        // wristVolts = 0;
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
		shoulderVolts.set(voltage);
	}

	@Override
    protected void setWristVoltageImpl(double voltage) {
		wristVolts.set(voltage);
	}

    @Override
    public void periodicallyCalled() {
        simulation.update(shoulderVolts.get(), wristVolts.get(), Settings.DT);
    }

    @Override
    public double getShoulderVelocityRadiansPerSecond() {
        return simulation.getShoulderVelocityRadiansPerSecond();
    }

    @Override
    public double getWristVelocityRadiansPerSecond() {
        return simulation.getWristVelocityRadiansPerSecond();
    }
}