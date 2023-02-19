package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import static com.stuypulse.robot.constants.Settings.Arm.*;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.DoubleJointedArmSim;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimArm extends Arm {

    private final DoubleJointedArmSim armSim;

    private final AngleController shoulderController; 
    private final AngleController wristController;

    private final ArmVisualizer armVisualizer;

    public SimArm() { 
        armSim = new DoubleJointedArmSim(
            // shoulder
            DCMotor.getNEO(2), Shoulder.REDUCTION, Shoulder.MOI, Units.inchesToMeters(Shoulder.LENGTH), Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 
            // wrist
            DCMotor.getNEO(1), Wrist.REDUCTION, Wrist.MOI, Units.inchesToMeters(Wrist.LENGTH),Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY

        );

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
        return Rotation2d.fromDegrees(armSim.getShoulderAngleDegrees());
    }

    @Override
    public Rotation2d getWristAngle() {
        return Rotation2d.fromDegrees(armSim.getWristAngleDegrees());
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
    
        armSim.setInput(shoulderOutput, wristOutput);
        armSim.update(Settings.DT);

        armVisualizer.setTargetAngles(getShoulderTargetAngle().getDegrees(), getWristTargetAngle().getDegrees());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());

        var targetState = getTargetState();
        SmartDashboard.putNumber("Arm/Shoulder/Target (deg)", targetState.getShoulderState().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Target (deg)", targetState.getWristState().getDegrees());

        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", shoulderController.getError().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", wristController.getError().toDegrees());

        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderOutput);
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristOutput);
    }
}