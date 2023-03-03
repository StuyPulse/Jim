package com.stuypulse.robot.subsystems.arm;

import static com.stuypulse.robot.constants.Motors.Arm.SHOULDER_LEFT_CONFIG;
import static com.stuypulse.robot.constants.Motors.Arm.SHOULDER_RIGHT_CONFIG;
import static com.stuypulse.robot.constants.Motors.Arm.WRIST_CONFIG;
import static com.stuypulse.robot.constants.Ports.Arm.SHOULDER_LEFT;
import static com.stuypulse.robot.constants.Ports.Arm.SHOULDER_RIGHT;
import static com.stuypulse.robot.constants.Ports.Arm.WRIST;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {

    private static int kDisableStatusFrame = 65535;

    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    protected ArmImpl() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);

        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        configureMotors();
    }

    private void configureMotors() {
        shoulderEncoder.setZeroOffset(0);
        wristEncoder.setZeroOffset(0);

        shoulderEncoder.setInverted(true);
        shoulderEncoder.setVelocityConversionFactor(Units.rotationsToDegrees(1));
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, kDisableStatusFrame);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, kDisableStatusFrame);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

        wristEncoder.setInverted(true);
        wristEncoder.setVelocityConversionFactor(Units.rotationsToDegrees(1));
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, kDisableStatusFrame);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, kDisableStatusFrame);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition()).minus(Shoulder.ZERO_ANGLE);
    }

    @Override
    public Rotation2d getRelativeWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Wrist.ZERO_ANGLE);
    }

    @Override
    protected void setShoulderVoltageImpl(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    @Override
    protected void setWristVoltageImpl(double voltage) {
        wrist.setVoltage(voltage);
    }

    @Override
    public void setCoast(boolean wristCoast, boolean shoulderCoast) {
        shoulderLeft.setIdleMode(shoulderCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        shoulderRight.setIdleMode(shoulderCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        wrist.setIdleMode(wristCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodicallyCalled() {
        SmartDashboard.putNumber("Arm/Shoulder/Let Bus Voltage (V)", shoulderLeft.getBusVoltage());
        SmartDashboard.putNumber("Arm/Shoulder/Right Bus Voltage (V)", shoulderRight.getBusVoltage());

        SmartDashboard.putNumber("Arm/Shoulder/Left Current (amps)", shoulderLeft.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Shoulder/Right Current (amps)", shoulderRight.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Wrist/Current (amps)", wrist.getOutputCurrent());

        SmartDashboard.putNumber("Arm/Shoulder/Raw Encoder Angle (rot)", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist/Raw Encoder Angle (rot)", wristEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Shoulder/Left Duty Cycle", shoulderLeft.get());
        SmartDashboard.putNumber("Arm/Shoulder/Right Duty Cycle", shoulderRight.get());
        SmartDashboard.putNumber("Arm/Wrist/Duty Cycle", wrist.get());

        SmartDashboard.putNumber("Arm/Shoulder/Encoder Velocity (deg per s)", shoulderEncoder.getVelocity());
        SmartDashboard.putNumber("Arm/Wrist/Encoder Velocity (deg per s)", wristEncoder.getVelocity());
    }
}