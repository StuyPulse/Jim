package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.ArmJoint;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.TwoJointArmSimulation;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimArm extends Arm {

    private ArmDynamics dynamics;
    private TwoJointArmSimulation simulation;

    private SmartNumber shoulderTargetAngle;
    private SmartNumber wristTargetAngle;

    private ArmVisualizer visualizer;

    public SimArm() {
        dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);
        simulation = new TwoJointArmSimulation(0, Math.PI, dynamics);

        shoulderTargetAngle = new SmartNumber("Arm/Shoulder Target (deg)", 0);
        wristTargetAngle = new SmartNumber("Arm/Wrist Target (deg)", 0);

        visualizer = new ArmVisualizer();

    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRadians(simulation.getShoulderPositionRadians());
    }

    @Override
    public Rotation2d getWristAngle() {
        return Rotation2d.fromRadians(simulation.getWristPositionRadians()).plus(getShoulderAngle());
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(shoulderTargetAngle.get());
    }

    @Override
    public Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
    }

    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        shoulderTargetAngle.set(angle.getDegrees());
    }

    @Override
    public void setTargetWristAngle(Rotation2d angle) {
        wristTargetAngle.set(angle.getDegrees());
    }

    @Override
    public void setFeedbackEnabled(boolean enabled) {
    }

    @Override
    public ArmVisualizer getVisualizer() {
        return visualizer;
    }

    @Override
    public void periodic() {
        simulation.update(0.0, 0.0, Settings.DT);

        visualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        visualizer.setTargetAngles(shoulderTargetAngle.get(), wristTargetAngle.get());
    }
    
}
