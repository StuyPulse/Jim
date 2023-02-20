package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.TwoJointArmSimulation;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimArm extends Arm {

    private final ArmDynamics dynamics;
    private final TwoJointArmSimulation simulation;

    private final AngleController shoulderController; 
    private final AngleController wristController;

    private final ArmVisualizer armVisualizer;

    public SimArm() { 
        dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);
        simulation = new TwoJointArmSimulation(-Math.PI/2, Math.PI/2, dynamics);

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                                    .setSetpointFilter(
                                        new AMotionProfile(
                                            Shoulder.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                                            Shoulder.MAX_VELOCITY.filtered(Math::toRadians).number()))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
                                    .setSetpointFilter(
                                        new AMotionProfile(
                                            Wrist.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                                            Wrist.MAX_VELOCITY.filtered(Math::toRadians).number()))
                                    .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRadians(simulation.getShoulderPositionRadians());
    }

    @Override
    public Rotation2d getWristAngle() {
        return Rotation2d.fromRadians(simulation.getWristPositionRadians()).plus(getShoulderAngle());
    }

    public ArmVisualizer getVisualizer() {
        return armVisualizer;
    }

    public void setFeedbackEnabled(boolean enabled) {
    }

    @Override
    public void periodic() {
        double shoulderOutput = shoulderController.update(Angle.fromRotation2d(getShoulderTargetAngle()), Angle.fromRotation2d(getShoulderAngle()));
        double wristOutput = wristController.update(Angle.fromRotation2d(getWristTargetAngle()), Angle.fromRotation2d(getWristAngle()));
    
        simulation.update(shoulderOutput, wristOutput, Settings.DT);

        armVisualizer.setTargetAngles(getShoulderTargetAngle().getDegrees(), getWristTargetAngle().getDegrees());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        Settings.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        Settings.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());

        var targetState = getTargetState();
        Settings.putNumber("Arm/Shoulder/Target (deg)", targetState.getShoulderState().getDegrees());
        Settings.putNumber("Arm/Wrist/Target (deg)", targetState.getWristState().getDegrees());

        Settings.putNumber("Arm/Shoulder/Error (deg)", shoulderController.getError().toDegrees());
        Settings.putNumber("Arm/Wrist/Error (deg)", wristController.getError().toDegrees());

        Settings.putNumber("Arm/Shoulder/Output (V)", shoulderOutput);
        Settings.putNumber("Arm/Wrist/Output (V)", wristOutput);
    }
}