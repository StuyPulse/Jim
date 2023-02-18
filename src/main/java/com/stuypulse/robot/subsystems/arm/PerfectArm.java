package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.util.ArmVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;

public class PerfectArm extends Arm {

    private final ArmVisualizer visualizer = new ArmVisualizer(); 

    private Rotation2d shoulderAngle = Rotation2d.fromDegrees(90);
    private Rotation2d wristAngle = Rotation2d.fromDegrees(-90);

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
    public ArmVisualizer getVisualizer() {
        return visualizer;
    }

    @Override
    public void periodic() {
        visualizer.setMeasuredAngles(shoulderAngle.getDegrees(), wristAngle.getDegrees());
        visualizer.setTargetAngles(shoulderAngle.getDegrees(), wristAngle.getDegrees());
    }
    

}