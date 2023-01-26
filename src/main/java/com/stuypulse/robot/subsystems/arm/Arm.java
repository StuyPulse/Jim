package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import com.stuypulse.robot.constants.Settings.Arm.Simulation.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Simulation.Wrist;
import com.stuypulse.robot.subsystems.IArm;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends IArm {
    
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    // todo: ask verit
    private final RelativeEncoder shoulderEncoder;
    private final RelativeEncoder wristEncoder;

    private final Controller shoulderController;
    private final Controller wristController;

    // degrees
    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle; 

    public Arm() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderLeft.getEncoder();
        shoulderEncoder.setPositionConversionFactor(SHOULDER_CONVERSION);
        wristEncoder = wrist.getEncoder();
        wristEncoder.setPositionConversionFactor(WRIST_CONVERSION);

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kA, Shoulder.Feedforward.kV).position()
                                    .add(new ArmFeedforward(Shoulder.Feedforward.kG))
                                    .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                                    .setSetpointFilter(new MotionProfile(SHOULDER_VEL_LIMIT, SHOULDER_ACC_LIMIT));
                                    // .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));;
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kA, Wrist.Feedforward.kV).position()
                                    .add(new ArmFeedforward(Wrist.Feedforward.kG))
                                    .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
                                    .setSetpointFilter(new MotionProfile(WRIST_VEL_LIMIT, WRIST_ACC_LIMIT));
                                    // .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));

        shoulderTargetAngle = new SmartNumber("Arm/Shoulder Target Angle", 0);
        wristTargetAngle = new SmartNumber("Arm/Wrist Target Angle", 0);
    
        configureMotors();
    }

    private void configureMotors() {
        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    @Override
    public double getShoulderDegrees() {
        return shoulderEncoder.getPosition();
    }

    @Override
    public double getWristDegrees() {
        return wristEncoder.getPosition();
    }

    @Override
    public void setTargetShoulderAngle(double angle) {
        shoulderTargetAngle.set(MathUtil.clamp(angle, Math.toDegrees(Shoulder.MINANGLE), Math.toDegrees(Shoulder.MAXANGLE)));
    }

    @Override
    public void setTargetWristAngle(double angle) {
        wristTargetAngle.set(MathUtil.clamp(angle, Math.toDegrees(Wrist.MINANGLE), Math.toDegrees(Wrist.MAXANGLE)));
    }

    @Override
    public void setTargetWristAngle(double angle, boolean clockwise) {
        double clamped = MathUtil.clamp(angle, Math.toDegrees(Wrist.MINANGLE), Math.toDegrees(Wrist.MAXANGLE));
        
        if (!clockwise) {
            wristTargetAngle.set(-clamped);
        } else {
            wristTargetAngle.set(clamped);
        }
    }

    @Override
    public boolean isShoulderAtAngle(double maxError) {
        return Math.abs(getShoulderDegrees() - shoulderTargetAngle.get()) < maxError;
    }

    @Override
    public boolean isWristAtAngle(double maxError) {
        return Math.abs(getWristDegrees() - wristTargetAngle.get()) < maxError;
    }

    public void moveShoulder(double angle) {
        shoulderTargetAngle.set(shoulderTargetAngle.get() + angle);
    }

    public void moveWrist(double angle) {
        wristTargetAngle.set(wristTargetAngle.get() + angle);
    }

    public double getAbsoluteWristAngle() {
        return 180 + getShoulderDegrees() - getWristDegrees();
    }

    private void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    private void runWrist(double voltage) {
        wrist.setVoltage(voltage);
    }

    public void execute() {
        double shoulderOutput = shoulderController.update(shoulderTargetAngle.get(), getShoulderDegrees());
        double wristOutput = wristController.update(wristTargetAngle.get(), getWristDegrees());

        runShoulder(shoulderOutput);
        runWrist(wristOutput);

        SmartDashboard.putNumber("Arm/Shoulder/Angle", getShoulderDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Angle", getWristDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Abs Angle", getAbsoluteWristAngle());
        
        SmartDashboard.putNumber("Arm/Shoulder/Output", shoulderOutput);
        SmartDashboard.putNumber("Arm/Wrist/Output", wristOutput);
    }
}
