package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.AngleMotionProfile;
import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.ArmJoint;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.TwoJointArmSimulation;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;

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
        simulation = new TwoJointArmSimulation(Math.PI/2, Math.PI/2, dynamics);

        shoulderTargetAngle = new SmartNumber("Arm/Shoulder Target (deg)", 90);
        wristTargetAngle = new SmartNumber("Arm/Wrist Target (deg)", -90);

        shoulderController = new AnglePIDController(5, 0, 1);
        wristController = new AnglePIDController(2.5, 0, 0.5);

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

    @Override
    public void periodic() {
        var u_ff = dynamics.feedforward(
            VecBuilder.fill(getShoulderTargetAngle().getRadians(), getRelativeWristTargetAngle().getRadians()),
            VecBuilder.fill(0, 0),
            VecBuilder.fill(0, 0));
        
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
