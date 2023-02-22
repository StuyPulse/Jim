package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;

public class PerfectArm extends Arm {

    private final ArmVisualizer visualizer;

    private Rotation2d shoulderAngle = Rotation2d.fromDegrees(90);
    private Rotation2d wristAngle = Rotation2d.fromDegrees(-90);

    public PerfectArm() {
        visualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return shoulderAngle;
    }

    @Override
    public Rotation2d getWristAngle() {
        return wristAngle;
    }

    @Override
    public void setFeedbackEnabled(boolean enabled) {
        
    }

    @Override
    public void setLimpWristEnabled(boolean enabled) {
        
    }

    @Override
    public ArmVisualizer getVisualizer() {
        return visualizer;
    }

    @Override
    public void periodic() {
        var targetState = getTargetState();
        shoulderAngle = targetState.getShoulderState();
        wristAngle = targetState.getWristState();

        visualizer.setMeasuredAngles(shoulderAngle.getDegrees(), wristAngle.getDegrees());
        visualizer.setTargetAngles(shoulderAngle.getDegrees(), wristAngle.getDegrees());
        visualizer.setFieldArm(Odometry.getInstance().getPose(), getState());
    }

}