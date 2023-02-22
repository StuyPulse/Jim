package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SimpleArm extends Arm {

    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final ArmVisualizer armVisualizer;

    private SmartBoolean feedbackEnable;
    private SmartBoolean limpWristEnable;

    public SimpleArm() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);

        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        configureMotors();

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD).setOutputFilter(x -> feedbackEnable.get() ? x : 0))
                                    .setSetpointFilter(
                                        new AMotionProfile(
                                            Shoulder.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                                            Shoulder.MAX_VELOCITY.filtered(Math::toRadians).number()));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
                                    .add(new AngleArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD).setOutputFilter(x -> feedbackEnable.get() ? x : 0))
                                    .setSetpointFilter(
                                        new AMotionProfile(
                                            Wrist.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                                            Wrist.MAX_VELOCITY.filtered(Math::toRadians).number()));

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));

        feedbackEnable = new SmartBoolean("Arm/Feedback Enable", true);

        setTargetState(getState());
    }

    private void configureMotors() {
        shoulderEncoder.setZeroOffset(0);
        wristEncoder.setZeroOffset(0);

        shoulderEncoder.setInverted(true);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        wristEncoder.setInverted(true);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition()).minus(Shoulder.ZERO_ANGLE);
    }

    private Rotation2d getRelativeWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Wrist.ZERO_ANGLE);
    }

    @Override
    public Rotation2d getWristAngle() {
        return getRelativeWristAngle().plus(getShoulderAngle());
    }

    private void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    private void runWrist(double voltage) {
        wrist.setVoltage(voltage);
    }

    @Override
    public ArmVisualizer getVisualizer() {
        return armVisualizer;
    }

    @Override
    public void setFeedbackEnabled(boolean enabled) {
        feedbackEnable.set(enabled);
    }

    @Override
    public void setLimpWristEnabled(boolean enabled) {
        limpWristEnable.set(enabled);
    }

    @Override
    public void periodic() {
        var targetState = getTargetState();
        double shoulderOutput = shoulderController.update(Angle.fromRotation2d(targetState.getShoulderState()), Angle.fromRotation2d(getShoulderAngle()));
        double wristOutput = wristController.update(Angle.fromRotation2d(targetState.getWristState()), Angle.fromRotation2d(getWristAngle()));

        runShoulder(shoulderOutput);
        
        if (limpWristEnable.get()) {
            wrist.setIdleMode(IdleMode.kCoast);
        } else {
            wrist.setIdleMode(IdleMode.kBrake);

            runWrist(wristOutput);
        }

        if (Settings.isDebug()) {
            armVisualizer.setTargetAngles(shoulderController.getSetpoint().toDegrees(), wristController.getSetpoint().toDegrees());
            armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
            armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());
        
            Settings.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
            Settings.putNumber("Arm/Shoulder/Raw Angle (deg)", Units.rotationsToDegrees(shoulderEncoder.getPosition()));

            Settings.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
            Settings.putNumber("Arm/Wrist/Raw Angle (deg)", Units.rotationsToDegrees(wristEncoder.getPosition()));

            Settings.putNumber("Arm/Shoulder/Target (deg)", targetState.getShoulderState().getDegrees());
            Settings.putNumber("Arm/Wrist/Target (deg)", targetState.getWristState().getDegrees());

            Settings.putNumber("Arm/Shoulder/Error (deg)", shoulderController.getError().toDegrees());
            Settings.putNumber("Arm/Wrist/Error (deg)", wristController.getError().toDegrees());

            Settings.putNumber("Arm/Shoulder/Output (V)", shoulderOutput);
            Settings.putNumber("Arm/Wrist/Output (V)", wristOutput);
        }
    }
}