package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {

    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final ArmDynamics dynamics;

    private final ArmVisualizer armVisualizer;

    private SmartBoolean feedbackEnable;

    public ArmImpl() {
        System.out.println("CREATING ARM IMPL ");
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

        dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);

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

    private Rotation2d getRelativeWristTargetAngle() {
        return getWristAngle().minus(getShoulderAngle());
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

        double shoulderVolts = 
            // u_ff.get(0, 0) +
            shoulderController.update(Angle.fromRotation2d(getShoulderTargetAngle()), Angle.fromRotation2d(getShoulderAngle()));
        
        double wristVolts =
            // u_ff.get(1, 0) +
            wristController.update(Angle.fromRotation2d(getWristTargetAngle()), Angle.fromRotation2d(getWristAngle()));

        runShoulder(shoulderVolts);
        runWrist(wristVolts);

        armVisualizer.setTargetAngles(getShoulderTargetAngle().getDegrees(), getWristTargetAngle().getDegrees());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());


        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Raw Angle (deg)", Units.rotationsToDegrees(shoulderEncoder.getPosition()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Raw Angle (deg)", Units.rotationsToDegrees(wristEncoder.getPosition()));

        var targetState = getTargetState();
        SmartDashboard.putNumber("Arm/Shoulder/Target (deg)", targetState.getShoulderState().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Target (deg)", targetState.getWristState().getDegrees());

        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", shoulderController.getError().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", wristController.getError().toDegrees());

        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderVolts);
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristVolts);
    }
}
