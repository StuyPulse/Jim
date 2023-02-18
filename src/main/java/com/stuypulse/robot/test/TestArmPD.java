package com.stuypulse.robot.test;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestArmPD extends SubsystemBase {
    
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final SmartNumber targetShoulderAngle;
    private final SmartNumber targetWristAngle;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    public TestArmPD() {  
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderController = new AngleArmFeedforward(Shoulder.Feedforward.kG)
            .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD));

        // shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
        //     .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
        //     .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
        //     .setSetpointFilter(new AMotionProfile(Shoulder.VEL_LIMIT, Shoulder.ACCEL_LIMIT));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
            .add(new AngleArmFeedforward(Wrist.Feedforward.kG));
            // .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD));
            // .setSetpointFilter(new AMotionProfile(Wrist.VEL_LIMIT, Wrist.ACCEL_LIMIT));

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        targetShoulderAngle = new SmartNumber("Arm/Target Shoulder Angle (deg)", getShoulderAngle().getDegrees());
        targetWristAngle = new SmartNumber("Arm/Target Wrist Angle (deg)", getWristAngle().getDegrees());

        configureMotors();
    }

    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition()).minus(Shoulder.ZERO_ANGLE);
    }

    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Wrist.ZERO_ANGLE).plus(getShoulderAngle());
    }

    private void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    private void runWrist(double voltage) {
        wrist.setVoltage(voltage);
    }

    public void configureMotors() {
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
    public void periodic() {
        double shoulderVolts = shoulderController.update(
            Angle.fromDegrees(targetShoulderAngle.get()),
            Angle.fromRotation2d(getShoulderAngle()));
        
        double wristVolts = wristController.update(
            Angle.fromDegrees(targetWristAngle.get()),
            Angle.fromRotation2d(getWristAngle()));

        SmartDashboard.putNumber("Arm/Shoulder Voltage (before clamp)", shoulderVolts);
        SmartDashboard.putNumber("Arm/Wrist Voltage (before clamp)", wristVolts);

        runShoulder(MathUtil.clamp(shoulderVolts, -3, 3));
        runWrist(MathUtil.clamp(wristVolts, -3, 3));


        SmartDashboard.putNumber("Arm/Shoulder Encoder", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Encoder", wristEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristAngle().getDegrees());
    }
}