package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.TwoJointArmSimulation;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimArm extends Arm {

    private ArmDynamics dynamics;
    private TwoJointArmSimulation simulation;

    private SmartNumber shoulderTargetAngle;
    private SmartNumber wristTargetAngle;

    private AngleController shoulderController;
    private AngleController wristController;

    private ArmVisualizer visualizer;

    public SimArm() {
        dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);
        simulation = new TwoJointArmSimulation(-Math.PI/2, Math.PI/2, dynamics);

        shoulderTargetAngle = new SmartNumber("Arm/Shoulder Target (deg)", 90);
        wristTargetAngle = new SmartNumber("Arm/Wrist Target (deg)", -90);

        shoulderController = new AnglePIDController(
            new SmartNumber("Arm/Shoulder/kP", 4), 
            0, 
            new SmartNumber("Arm/Shoulder/kD", 0.8));

        wristController =  new AnglePIDController(
            new SmartNumber("Arm/Wrist/kP", 4), 
            0, 
            new SmartNumber("Arm/Wrist/kD", 0.8));

        visualizer = new ArmVisualizer();
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRadians(simulation.getShoulderPositionRadians());
    }

    @Override
    public Rotation2d getWristAngle() {
        return getRelativeWristTargetAngle().plus(getShoulderAngle());
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(shoulderTargetAngle.get());
    }

    @Override
    public Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
    }

    public Rotation2d getRelativeWristTargetAngle() {
        // return getWristAngle().minus(getShoulderAngle());
        return Rotation2d.fromRadians(simulation.getWristPositionRadians());
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


    private Rotation2d lastShoulderAngle;
    private Rotation2d lastWristAngle;
    
    private double lastShoulderVelocity = Double.NaN;
    private double lastWristVelocity = Double.NaN;


    @Override
    public void periodic() {

        var u_ff = VecBuilder.fill(0, 0);

        if (lastShoulderAngle != null && lastWristAngle != null) {
            lastShoulderVelocity = getShoulderTargetAngle().minus(lastShoulderAngle).getRadians() / Settings.DT;
            lastWristVelocity = getWristTargetAngle().minus(lastWristAngle).getRadians() / Settings.DT;
        }

        if (!Double.isNaN(lastShoulderVelocity) && !Double.isNaN(lastWristVelocity)) {
            double currentShoulderVelocity = getShoulderTargetAngle().minus(lastShoulderAngle).getRadians() / Settings.DT;
            double currentWristVelocity = getWristTargetAngle().minus(lastWristAngle).getRadians() / Settings.DT;
            
            u_ff = dynamics.feedforward(
                VecBuilder.fill(getShoulderTargetAngle().getRadians(), getRelativeWristTargetAngle().getRadians()),
                VecBuilder.fill(currentShoulderVelocity, currentWristVelocity),
                VecBuilder.fill(
                    (currentShoulderVelocity - lastShoulderVelocity)/ Settings.DT, 
                    (currentWristVelocity - lastWristVelocity) / Settings.DT));

            lastShoulderVelocity = currentShoulderVelocity;
            lastWristVelocity = currentWristVelocity;
        }

        lastWristAngle = getWristTargetAngle();
        lastShoulderAngle = getShoulderTargetAngle();
        
        u_ff = VecBuilder.fill(
            MathUtil.clamp(u_ff.get(0, 0), -12, 12),
            MathUtil.clamp(u_ff.get(1, 0), -12, 12));

        SmartDashboard.putNumber("Arm/Wrist Voltage", u_ff.get(0, 0));
        SmartDashboard.putNumber("Arm/Shoulder Voltage", u_ff.get(1, 0));

        simulation.update( 
            u_ff.get(0, 0) +
            shoulderController.update(Angle.fromRotation2d(getShoulderTargetAngle()), Angle.fromRotation2d(getShoulderAngle())),
            u_ff.get(1, 0) + 
            wristController.update(Angle.fromRotation2d(getWristTargetAngle()), Angle.fromRotation2d(getWristAngle())),
            Settings.DT);

        visualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        visualizer.setTargetAngles(shoulderTargetAngle.get(), wristTargetAngle.get());
    }
    
}
