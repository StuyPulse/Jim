package com.stuypulse.robot.test;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ArmDynamics;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestArm extends SubsystemBase {

    private static final SmartBoolean SETPOINT_CONTROL = new SmartBoolean("Arm/Setpoint Control", false);
    
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final SmartNumber targetShoulderAngle;
    private final SmartNumber targetWristAngle;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final ArmDynamics dynamics;

    public TestArm() {  
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        // shoulderController = new AngleArmFeedforward(Shoulder.Feedforward.kG)
        //     .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD));

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
            .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
            .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD));
        //     .setSetpointFilter(new AMotionProfile(Shoulder.VEL_LIMIT, Shoulder.ACCEL_LIMIT));

        // wristController = new AngleArmFeedforward(Wrist.Feedforward.kG)
        //     .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
            .add(new AngleArmFeedforward(Wrist.Feedforward.kG))
            .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD));
            // .setSetpointFilter(new AMotionProfile(Wrist.VEL_LIMIT, Wrist.ACCEL_LIMIT));

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        targetShoulderAngle = new SmartNumber("Arm/Target Shoulder Angle (deg)", getShoulderAngle().getDegrees());
        targetWristAngle = new SmartNumber("Arm/Target Wrist Angle (deg)", getWristAngle().getDegrees());

        dynamics = new ArmDynamics(Shoulder.JOINT, Wrist.JOINT);

        configureMotors();
    }

    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition()).minus(Shoulder.ZERO_ANGLE);
    }

    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Wrist.ZERO_ANGLE).plus(getShoulderAngle());
    }

    public Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(targetShoulderAngle.get());
    }

    public Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(targetWristAngle.get());
    }

    private Rotation2d getRelativeWristTargetAngle() {
        return getWristAngle().minus(getShoulderAngle());
    }

    public void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);

        SmartDashboard.putNumber("Arm/Shoulder Voltage", voltage);
    }

    public void runWrist(double voltage) {
        wrist.setVoltage(voltage);

        SmartDashboard.putNumber("Arm/Wrist Voltage", voltage);
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
    
    private Rotation2d lastShoulderAngle;
    private Rotation2d lastWristAngle;
    
    private double lastShoulderVelocity = Double.NaN;
    private double lastWristVelocity = Double.NaN;

    @Override
    public void periodic() {
        if (SETPOINT_CONTROL.get()) {
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
                shoulderController.update(Angle.fromDegrees(targetShoulderAngle.get()), Angle.fromRotation2d(getShoulderAngle()));
            
            double wristVolts =
                // u_ff.get(1, 0) +
                wristController.update(Angle.fromDegrees(targetWristAngle.get()), Angle.fromRotation2d(getWristAngle()));
    
    
            runShoulder(shoulderVolts);
            runWrist(wristVolts);
        }

        SmartDashboard.putNumber("Arm/Shoulder Encoder", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Encoder", wristEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristAngle().getDegrees());
    }
}