package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.util.ArmVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;

public class PerfectArm extends Arm {

    private Rotation2d shoulderAngle = Rotation2d.fromDegrees(90);
    private Rotation2d wristAngle = Rotation2d.fromDegrees(-90);

    @Override
    public Rotation2d getShoulderAngle() {
        // TODO Auto-generated method stub
        return shoulderAngle;
    }

    @Override
    public Rotation2d getWristAngle() {
        // TODO Auto-generated method stub
        return wristAngle;
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        // TODO Auto-generated method stub
        return shoulderAngle    ;
    }

    @Override
    public Rotation2d getWristTargetAngle() {
        // TODO Auto-generated method stub
        return wristAngle;
    }

    @Override
    public void setTargetShoulderAngle(Rotation2d angle) {
        shoulderAngle = angle;
        
    }

    @Override
    public void setTargetWristAngle(Rotation2d angle) {
wristAngle = angle;
    }

    @Override
    public void setFeedbackEnabled(boolean enabled) {
        
        
    }

    // @Override
    // public ArmVisualizer getVisualizer() {
    //     // TODO Auto-generated method stub
    //     return vis;
    // }

    ArmVisualizer vis = new ArmVisualizer(); 

    @Override
    public void periodic() {
        vis.setMeasuredAngles(shoulderAngle.getDegrees(), wristAngle.getDegrees());
       vis.setTargetAngles(shoulderAngle.getDegrees(), wristAngle.getDegrees());
    }
    

}