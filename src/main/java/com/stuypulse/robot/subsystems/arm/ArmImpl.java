package com.stuypulse.robot.subsystems.arm;

import static com.stuypulse.robot.constants.Motors.Arm.SHOULDER_LEFT_CONFIG;
import static com.stuypulse.robot.constants.Motors.Arm.SHOULDER_RIGHT_CONFIG;
import static com.stuypulse.robot.constants.Motors.Arm.WRIST_CONFIG;
import static com.stuypulse.robot.constants.Ports.Arm.SHOULDER_LEFT;
import static com.stuypulse.robot.constants.Ports.Arm.SHOULDER_RIGHT;
import static com.stuypulse.robot.constants.Ports.Arm.WRIST;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.WrappedAbsoluteEncoder;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.TimedMovingAverage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {

    private static int kDisableStatusFrame = 65535;

    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final WrappedAbsoluteEncoder wristEncoder;

    private final IFilter wristVelocityFilter;
    private final IFilter shoulderVelocityFilter;

    protected ArmImpl() {
        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);

        wristEncoder = new WrappedAbsoluteEncoder(wrist.getAbsoluteEncoder(Type.kDutyCycle));

        // Probably helps?
        wristVelocityFilter = new TimedMovingAverage(0.1);
        shoulderVelocityFilter = new TimedMovingAverage(0.1);

        configureMotors();
    }

    private void configureMotors() {
        shoulderEncoder.setZeroOffset(0);
        wristEncoder.getEncoder().setZeroOffset(0);

        shoulderEncoder.setInverted(true);
        shoulderEncoder.setVelocityConversionFactor(Units.rotationsToRadians(1));
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, kDisableStatusFrame);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, kDisableStatusFrame);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

        wristEncoder.getEncoder().setInverted(true);
        wristEncoder.getEncoder().setVelocityConversionFactor(Units.rotationsToRadians(1));
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, kDisableStatusFrame);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, kDisableStatusFrame);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

        SHOULDER_LEFT_CONFIG.configure(shoulderLeft);
        SHOULDER_RIGHT_CONFIG.configure(shoulderRight);
        WRIST_CONFIG.configure(wrist);
    }

    @Override
    public void setShoulderIdleMode(IdleMode mode) {
        shoulderRight.setIdleMode(mode);
        shoulderLeft.setIdleMode(mode);
    }

    @Override
    public void setWristIdleMode(IdleMode mode) {
        wrist.setIdleMode(mode);
    }

    @Override
    public double getShoulderVelocityRadiansPerSecond() {
        return shoulderVelocityFilter.get(shoulderEncoder.getVelocity());
    }

    // private boolean isShoulderStalling() {
    //     double appliedShoulderVoltage = 
    //         Math.max(
    //             shoulderRight.getAppliedOutput() * shoulderRight.getBusVoltage(),
    //             shoulderLeft.getAppliedOutput() * shoulderLeft.getBusVoltage(),
    //         );

    //     return shoulderEncoder.getVelocity() < Shoulder.STALLING_VELOCITY.doubleValue() && shoulderVolts > Shoulder.STALLING_VOLTAGE.doubleValue() ||
    //             wrist.getOutputCurrent() > Shoulder.STALLING_CURRENT.doubleValue();
    // }

    // private boolean isWristStalling() {
    //     return wristEncoder.getVelocity() < Wrist.STALLING_VELOCITY.doubleValue() && wristVolts > Wrist.STALLING_VOLTAGE.doubleValue() ||
    //             shoulderLeft.getOutputCurrent() > Wrist.STALLING_CURRENT.doubleValue() || 
    //             shoulderRight.getOutputCurrent() > Wrist.STALLING_CURRENT.doubleValue();
    // }

    @Override
    public void periodicallyCalled() {
        SmartDashboard.putNumber("Arm/Shoulder/Let Bus Voltage (V)", shoulderLeft.getBusVoltage());
        SmartDashboard.putNumber("Arm/Shoulder/Right Bus Voltage (V)", shoulderRight.getBusVoltage());
        SmartDashboard.putNumber("Arm/Wrist/Bus Voltage (V)", wrist.getBusVoltage());

        SmartDashboard.putNumber("Arm/Shoulder/Left Current (amps)", shoulderLeft.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Shoulder/Right Current (amps)", shoulderRight.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Wrist/Current (amps)", wrist.getOutputCurrent());
        
        // if (wristIsStalling()) {
        //     setWristVoltageImpl(WRIST);
        // }

        // if (armIsStalling()) {
        //     shoulderVolts = 0;
        // }

        // runShoulder(shoulderVolts);
        // runWrist(wristVolts);

        SmartDashboard.putNumber("Arm/Wrist/Looped Encoder Angle (deg)", Units.rotationsToDegrees(wristEncoder.getRotations()));

        SmartDashboard.putNumber("Arm/Shoulder/Raw Encoder Angle (rot)", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist/Raw Encoder Angle (rot)", wristEncoder.getEncoder().getPosition());

        SmartDashboard.putNumber("Arm/Shoulder/Left Duty Cycle", shoulderLeft.get());
        SmartDashboard.putNumber("Arm/Shoulder/Right Duty Cycle", shoulderRight.get());
        SmartDashboard.putNumber("Arm/Wrist/Duty Cycle", wrist.get());
    }

    @Override
    public double getShoulderAngleRadians() {
        return Units.rotationsToRadians(shoulderEncoder.getPosition());
    }

    @Override
    protected double getRelativeWristAngleRadians() {
        return Units.rotationsToRadians(wristEncoder.getRotations());
    }

    @Override
    public double getWristVelocityRadiansPerSecond() {
        return wristVelocityFilter.get(wristEncoder.getEncoder().getVelocity());
    }

    @Override
    protected void setShoulderVoltageImpl(double voltage) {
        shoulderRight.setVoltage(voltage);
        shoulderLeft.setVoltage(voltage);
    }

    @Override
    protected void setWristVoltageImpl(double voltage) {
        wrist.setVoltage(voltage);
    }
}