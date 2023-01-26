package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends IArm {
    
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final RelativeEncoder shoulderEncoder;
    private final RelativeEncoder wristEncoder;

    private final Controller shoulderController;
    private final Controller wristController;

    private final SmartAngle shoulderTargetAngle;
    private final SmartAngle wristTargetAngle; 

    //shut up amber
    public Arm() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderLeft.getEncoder();
        shoulderEncoder.setPositionConversionFactor(SHOULDER_CONVERSION);
        wristEncoder = wrist.getEncoder();
        wristEncoder.setPositionConversionFactor(WRIST_CONVERSION);

        // TODO: Add ArmPIDController
        shoulderController = new PIDController(ShoulderFeedback.P, ShoulderFeedback.I, ShoulderFeedback.D);
        wristController = new PIDController(WristFeedback.P, WristFeedback.I, WristFeedback.D);

        shoulderTargetAngle = new SmartAngle("Arm/Shoulder Target Angle", Angle.kZero);
        wristTargetAngle = new SmartAngle("Arm/Wrist Target Angle", Angle.kZero);

        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    public Angle getShoulderAngle() {
        return Angle.fromDegrees(shoulderEncoder.getPosition());
    }

    public Angle getWristAngle() {
        return Angle.fromDegrees(wristEncoder.getPosition());
    }

    public void setTargetShoulderAngle(Angle angle) {
        shoulderTargetAngle.set(angle);
    }

    public void setTargetWristAngle(Angle angle) {
        wristTargetAngle.set(angle);
    }

    public boolean isShoulderAtAngle(double maxError) {
        return Math.abs(getShoulderAngle().add(shoulderTargetAngle.get().negative()).toDegrees()) < maxError;
    }

    public boolean isWristAtAngle(double maxError) {
        return Math.abs(getWristAngle().add(wristTargetAngle.get().negative()).toDegrees()) < maxError;
    }

    public void moveShoulder(Angle angle) {
        shoulderTargetAngle.set(shoulderTargetAngle.get().add(angle));
    }

    public void moveWrist(Angle angle) {
        wristTargetAngle.set(wristTargetAngle.get().add(angle));
    }

    public Angle getAbsoluteWristAngle() {
        return Angle.k180deg.add(getShoulderAngle()).add(getWristAngle().negative());
    }

    private void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    private void runWrist(double voltage) {
        wrist.setVoltage(voltage);
    }

    public void execute() {
        double shoulderOutput = shoulderController.update(shoulderTargetAngle.get().toDegrees(), getShoulderAngle().toDegrees());
        double wristOutput = wristController.update(wristTargetAngle.get().toDegrees(), getWristAngle().toDegrees());

        runShoulder(shoulderOutput);
        runWrist(wristOutput);

        SmartDashboard.putNumber("Arm/Shoulder/Angle", getShoulderAngle().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Angle", getWristAngle().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Abs Angle", getAbsoluteWristAngle().toDegrees());
        
        SmartDashboard.putNumber("Arm/Shoulder/Output", shoulderOutput);
        SmartDashboard.putNumber("Arm/Wrist/Output", wristOutput);
    }
}