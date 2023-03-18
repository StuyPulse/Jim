package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.MotionProfile;
import com.stuypulse.robot.util.ArmEncoderFeedforward;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

    
/**
  */ 
public abstract class Arm extends SubsystemBase {

    // Singleton
    private static final Arm instance;

    static {
        if (RobotBase.isSimulation())
            instance = new PerfectArm();
        else if (Settings.ROBOT == Robot.JIM)
            instance = new ArmImpl();
        else
            instance = new PerfectArm();
    }

    public static Arm getInstance() {
        return instance;
    }

    /* ARM VARIABLES */

    // represents a goal for the arm (separate from the profiled setpoints)
    private final SmartNumber shoulderTargetDegrees;
    private final SmartNumber wristTargetDegrees;

    // controllers for each joint
    private final Controller shoulderController;
    private final Controller wristController;

    // Mechanism2d visualizer
    private final ArmVisualizer armVisualizer;

    // Voltage overrides (used when present)
    private Optional<Double> wristVoltageOverride;
    private Optional<Double> shoulderVoltageOverride;

    protected Arm() {
        shoulderTargetDegrees = new SmartNumber("Arm/Shoulder/Target Angle (deg)", -90);
        wristTargetDegrees = new SmartNumber("Arm/Wrist/Target Angle (deg)", +90);

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).position()
            .add(new ArmEncoderFeedforward(Shoulder.Feedforward.kG, Math::cos))
            // .add(new ArmDriveFeedforward(Shoulder.Feedforward.kG, SwerveDrive.getInstance()::getForwardAccelerationGs))
            .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
            .setSetpointFilter(
                new MotionProfile(
                    Shoulder.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                    Shoulder.MAX_ACCELERATION.filtered(Math::toRadians).number()))
            .setOutputFilter(x -> shoulderVoltageOverride.orElse(x))
        ;
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).position()
            .add(new ArmEncoderFeedforward(Wrist.Feedforward.kG, Math::cos))
            .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
            .setSetpointFilter(
                new MotionProfile(
                    Wrist.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                    Wrist.MAX_ACCELERATION.filtered(Math::toRadians).number()))
            .setOutputFilter(x -> wristVoltageOverride.orElse(isWristControlEnabled() ? x : 0))
        ;

        wristVoltageOverride = Optional.of(0.0);
        shoulderVoltageOverride = Optional.of(0.0);

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));
    }

    // Arm Control Overrides

    private final boolean isWristControlEnabled() {
        final double velocity = Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond());
        return Math.abs(velocity) < Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.get();
    }

    // validate arm angles
    private final static double normalizeShoulderAngleDegrees(double degrees) {
        return MathUtil.inputModulus(degrees, Shoulder.MIN_CONTROL_ANGLE, Shoulder.MAX_CONTROL_ANGLE);
    }

    private final static double normalizeWristAngleDegrees(double degrees) {
        return MathUtil.inputModulus(degrees, Wrist.MIN_CONTROL_ANGLE, Wrist.MAX_CONTROL_ANGLE);
    }

    // returns normalized shoulder target in radians
    public final double getShoulderTargetAngleRadians() {
        return Math.toRadians(normalizeShoulderAngleDegrees(shoulderTargetDegrees.get()));
    }
    
    // returns normalized wrist target in radians
    public final double getWristTargetAngleRadians() {
        return Math.toRadians(normalizeWristAngleDegrees(wristTargetDegrees.get()));
    }

    // sets target shoulder angle, normalizes input angle
    public final void setShoulderTargetAngle(Rotation2d angle) {
        shoulderVoltageOverride = Optional.empty();
        shoulderTargetDegrees.set(normalizeShoulderAngleDegrees(angle.getDegrees())); // probably redundant to normalize here
    }

    // sets target wrist angle, normalizes input angle
    public final void setWristTargetAngle(Rotation2d angle) {
        wristVoltageOverride = Optional.empty();
        wristTargetDegrees.set(normalizeWristAngleDegrees(angle.getDegrees())); // probably redundant to normalize here
    }

    // Check if shoulder is at target state (not profiled, "in between" setpoint)
    public final boolean isShoulderAtTarget(double epsilonDegrees) {
        return Math.abs(getShoulderTargetAngleRadians() - getShoulderAngleRadians()) < epsilonDegrees;
    }

    // check if wrist is at target state (not profiled, "in between" setpoint)
    public final boolean isWristAtTarget(double epsilonDegrees) {
        return Math.abs(getWristTargetAngleRadians() - getWristAngleRadians()) < epsilonDegrees;
    }

    // Read angle measurements

    // reads the angle of the shoulder from with zero as horizontal and positive as ccw
    public abstract double getShoulderAngleRadians();

    // reads the angle of the wrist relative to arm, with zero as orthogonal and positive
    // as ccw
    protected abstract double getRelativeWristAngleRadians();

    public final double getWristAngleRadians() {
        // may want to normalize shoulder angle here, but it physically cannot go out of bounds
        return getShoulderAngleRadians() + getRelativeWristAngleRadians();
    }

    // reads 
    public abstract double getShoulderVelocityRadiansPerSecond();

    public abstract double getWristVelocityRadiansPerSecond();

    // set a shoulder voltage override
    public void setShoulderVoltage(double voltage) {
        shoulderVoltageOverride = Optional.of(voltage);
    }

    // set a wrist voltage override
    public void setWristVoltage(double voltage) {
        wristVoltageOverride = Optional.of(voltage);
    }

    // Feed a voltage to the hardware layer
    protected abstract void setShoulderVoltageImpl(double voltage);
    protected abstract void setWristVoltageImpl(double voltage);

    // set shoulder idle mode mode
    public void setShoulderIdleMode(IdleMode mode) {}

    public final void enableShoulderBrakeMode() {
        setShoulderIdleMode(IdleMode.kBrake);
    }
    public final void enableShoulderCoastMode() {
        setShoulderIdleMode(IdleMode.kCoast);
    }

    // set wrist idle mode
    public void setWristIdleMode(IdleMode mode) {}

    public final void enableWristBrakeMode() {
        setWristIdleMode(IdleMode.kBrake);
    }
    public final void enableWristCoastMode() {
        setWristIdleMode(IdleMode.kCoast);
    }

    // Arm Visualizer
    public final ArmVisualizer getVisualizer() {
        return armVisualizer;
    }

    private void validateArmTargets() {
        double shoulderDegrees = Math.toDegrees(getShoulderAngleRadians());
        if (shoulderDegrees > Shoulder.MAX_SHOULDER_ANGLE.get()) {
            setShoulderTargetAngle(Rotation2d.fromDegrees(Shoulder.MAX_SHOULDER_ANGLE.get()));
        }

        else if (shoulderDegrees < (180 - Shoulder.MAX_SHOULDER_ANGLE.get())) {
            setShoulderTargetAngle(Rotation2d.fromDegrees(180 - Shoulder.MAX_SHOULDER_ANGLE.get()));
        }
    }

    @Override
    public final void periodic() {
        validateArmTargets();

        // Run control loops on validated target angles
        shoulderController.update(getShoulderTargetAngleRadians(), getShoulderAngleRadians());
        wristController.update(getWristTargetAngleRadians(), getWristAngleRadians());

        setWristVoltageImpl(wristController.getOutput());
        setShoulderVoltageImpl(shoulderController.getOutput());
        
        armVisualizer.setTargetAngles(Math.toDegrees(shoulderController.getSetpoint()), Math.toDegrees(wristController.getSetpoint()));
        armVisualizer.setMeasuredAngles(Math.toDegrees(getShoulderAngleRadians()), Math.toDegrees(getWristAngleRadians()));
        // armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg) ", Math.toDegrees(getShoulderAngleRadians()));
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)",Math.toDegrees(shoulderController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", Math.toDegrees(shoulderController.getError()));
        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Shoulder/Velocity (deg per s)", Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", Math.toDegrees(getWristAngleRadians()));
        SmartDashboard.putNumber("Arm/Wrist/Relative Angle (deg)", Math.toDegrees(getRelativeWristAngleRadians()));
        SmartDashboard.putNumber("Arm/Wrist/Setpoint (deg)", Math.toDegrees(wristController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", Math.toDegrees(wristController.getError()));
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist/Velocity (deg per s)", Units.radiansToDegrees(getWristVelocityRadiansPerSecond()));
        SmartDashboard.putBoolean("Arm/Wrist/Control Enabled", isWristControlEnabled());

        periodicallyCalled();
    }

    public void periodicallyCalled() {}
}