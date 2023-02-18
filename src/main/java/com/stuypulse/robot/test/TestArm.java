package com.stuypulse.robot.test;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.network.SmartNumber;
import static com.stuypulse.robot.constants.Motors.Arm.*;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestArm extends SubsystemBase {
    
    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    public TestArm() {        
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        configureMotors();
    }

    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition()).minus(Shoulder.ZERO_ANGLE);
    }

    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Wrist.ZERO_ANGLE).plus(getShoulderAngle());
    }

    public void runShoulder(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    public void runWrist(double voltage) {
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
        SmartDashboard.putNumber("Arm/Shoulder Voltage", shoulderLeft.get() * 12);
        SmartDashboard.putNumber("Arm/Wrist Voltage", wrist.get() * 12);

        SmartDashboard.putNumber("Arm/Shoulder Encoder", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Encoder", wristEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristAngle().getDegrees());
    }
}