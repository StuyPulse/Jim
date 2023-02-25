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
import com.stuypulse.robot.util.AngleVelocity;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {

    private final AngleVelocity wristVelocity;
    private final AngleVelocity shoulderVelocity;

    private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;

    private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    private final BStream wristStalling;
    private final BStream armStalling;

    public ArmImpl() {
        wristVelocity = new AngleVelocity();
        shoulderVelocity = new AngleVelocity();

        shoulderLeft = new CANSparkMax(SHOULDER_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(SHOULDER_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);

        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        wristStalling = BStream.create(this::getWristMomentarilyStalling).filtered(new BDebounce.Rising(Wrist.STALLING_TIME.doubleValue()));
        armStalling = BStream.create(this::getShoulderMomentarilyStalling).filtered(new BDebounce.Rising(Shoulder.STALLING_TIME.doubleValue()));

        configureMotors();
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

    @Override
    public Rotation2d getRelativeWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Wrist.ZERO_ANGLE);
    }

    @Override
    protected void setShoulderVoltage(double voltage) {
        shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
    }

    @Override
    protected void setWristVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }

    @Override
    public boolean getShoulderStalling() {
        return armStalling.get();
    }

    @Override
    public boolean getWristStalling() {
        return wristStalling.get();
    }

    private boolean getShoulderMomentarilyStalling() {
        return shoulderVelocity.getOutput() < Shoulder.STALLING_VELOCITY.doubleValue() && shoulderLeft.getAppliedOutput() > Shoulder.MIN_DUTY_CYCLE.doubleValue() ||
            shoulderLeft.getOutputCurrent() > Wrist.STALLING_CURRENT.doubleValue() || 
            shoulderRight.getOutputCurrent() > Wrist.STALLING_CURRENT.doubleValue();
    }

    private boolean getWristMomentarilyStalling() {
        return wristVelocity.getOutput() < Wrist.STALLING_VELOCITY.doubleValue() && wrist.getOutputCurrent() > Wrist.MIN_DUTY_CYCLE.doubleValue() ||
            wrist.getOutputCurrent() > Wrist.STALLING_CURRENT.doubleValue();
    }

    @Override
    public void setCoast(boolean coast) {
        shoulderLeft.setIdleMode(coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        shoulderRight.setIdleMode(coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        wrist.setIdleMode(coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodicallyCalled() {
        shoulderVelocity.update(Angle.fromRotation2d(getShoulderAngle()));
        wristVelocity.update(Angle.fromRotation2d(getWristAngle()));

        if (getShoulderStalling()) {
            setShoulderVoltage(0);
            setTargetState(new ArmState(getShoulderAngle(), getWristTargetAngle()));
        }

        if (getWristStalling()) {
            setWristVoltage(0);
            setTargetState(new ArmState(getShoulderTargetAngle(), getWristAngle()));
        }


        SmartDashboard.putNumber("Arm/Shoulder/Raw Encoder Angle (rot)", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist/Raw Encoder Angle (rot)", wristEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Shoulder/Left Duty Cycle", shoulderLeft.get());
        SmartDashboard.putNumber("Arm/Shoulder/Right Duty Cycle", shoulderRight.get());
        SmartDashboard.putNumber("Arm/Wrist/Duty Cycle", wrist.get());
    }
}
