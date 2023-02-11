package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.util.ArmVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;

public class NoArm extends Arm {

    ArmVisualizer visualizer = new ArmVisualizer();

    public ArmVisualizer getVisualizer() {
        return visualizer;
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d getWristAngle() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d getWristTargetAngle() {
        return new Rotation2d();
    }


    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        
    }

    @Override
    public void setTargetWristAngle(Rotation2d angle) {
        
    }


    public void setFeedbackEnabled(boolean enabled) {
        
    }
    
}
