package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.FieldArm2d;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {

    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle; 

    private final ArmVisualizer armVisualizer;

    private final FieldArm2d fieldArm;

    private SmartBoolean feedbackEnable;

    public ArmImpl() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        shoulderEncoder.setZeroOffset(Shoulder.ANGLE_OFFSET);
        wristEncoder.setZeroOffset(Wrist.ANGLE_OFFSET);

        configureMotors();

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD).setOutputFilter(x -> feedbackEnable.get() ? x : 0))
                                    .setSetpointFilter(new AMotionProfile(Shoulder.VEL_LIMIT, Shoulder.ACCEL_LIMIT));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD).setOutputFilter(x -> feedbackEnable.get() ? x : 0))
                                    .setSetpointFilter(new AMotionProfile(Wrist.VEL_LIMIT, Wrist.ACCEL_LIMIT));

        shoulderTargetAngle = new SmartNumber("Arm/Shoulder Target Angle (deg)", 0);
        wristTargetAngle = new SmartNumber("Arm/Wrist Target Angle (deg)", 0);

        armVisualizer = new ArmVisualizer();

        fieldArm = new FieldArm2d(Odometry.getInstance().getField().getObject("Field Arm"));

        feedbackEnable = new SmartBoolean("Arm/Feedback Enable", true);
    }

    private void configureMotors() {
        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(SLMath.map(shoulderEncoder.getPosition(), 0, 1, -180, 180));
    }

    @Override
    public Rotation2d getWristAngle() {
        return Rotation2d.fromDegrees(SLMath.map(wristEncoder.getPosition(), 0, 1, -180, 180));
    }

    @Override
    public Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetAngle.get());
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

    private void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    private void runWrist(double voltage) {
        wrist.setVoltage(voltage);
    }

    public ArmVisualizer getVisualizer() {
        return armVisualizer;
    }

    public void setFeedbackEnabled(boolean enabled) {
        feedbackEnable.set(enabled);
    }

    @Override
    public void periodic() {
        double shoulderOutput = shoulderController.update(Angle.fromDegrees(shoulderTargetAngle.get()), Angle.fromRotation2d(getShoulderAngle()));
        double wristOutput = wristController.update(Angle.fromDegrees(wristTargetAngle.get()), Angle.fromRotation2d(getWristAngle()));

        runShoulder(shoulderOutput);
        runWrist(wristOutput);

        armVisualizer.setTargetAngles(shoulderTargetAngle.get(), wristTargetAngle.get());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());

        fieldArm.update(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        
        SmartDashboard.putNumber("Arm/Shoulder/Output", shoulderOutput);
        SmartDashboard.putNumber("Arm/Wrist/Output", wristOutput);
    }
}
