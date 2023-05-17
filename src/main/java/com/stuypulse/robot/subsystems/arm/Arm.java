/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ArmDriveFeedforward;
import com.stuypulse.robot.util.ArmEncoderAngleFeedforward;
import com.stuypulse.robot.util.ArmEncoderFeedforward;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Double jointed arm controlled by two motion profiled PID controllers.
 *
 * Available control "modes":
 * - setpoint control (PID+FF controllers are used) (shoulder is not allowed
 * above maximum shoulder angle)
 * - limp mode (controller output is overriden to be zero)
 * - voltage override ("force" feeds a voltage to the motor)
 */
public abstract class Arm extends SubsystemBase {

    // Singleton
    private static final Arm instance;

    static {
        if (RobotBase.isReal() && Settings.ROBOT == Robot.JIM)
            instance = new ArmImpl();
        else
            instance = new SimArm();
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
    private final AngleController wristController;

    // Mechanism2d visualizer
    private final ArmVisualizer armVisualizer;

    private SmartNumber shoulderVelocityFeedbackDebounce;
    private SmartNumber shoulderVelocityFeedbackCutoff;

    private boolean pieceGravityCompensation;

    private final SmartNumber shoulderMaxVelocity;
    private final SmartNumber shoulderMaxAcceleration;

    private final SmartNumber wristMaxVelocity;
    private final SmartNumber wristMaxAcceleration;

    private class GamePiecekG extends Number {
        @Override
        public double doubleValue() {
            if (!pieceGravityCompensation) {
                return Shoulder.Feedforward.kGEmpty.doubleValue();
            }

            switch (Manager.getInstance().getGamePiece()) {
                case CONE_TIP_IN:
                    return Shoulder.Feedforward.kGCone.doubleValue();
                case CONE_TIP_OUT:
                    return Shoulder.Feedforward.kGCone.doubleValue();
                case CONE_TIP_UP:
                    return Shoulder.Feedforward.kGCone.doubleValue();
                case CUBE:
                    return Shoulder.Feedforward.kGCube.doubleValue();
                default:
                    return Shoulder.Feedforward.kGEmpty.doubleValue();
            }
        }

        @Override
        public float floatValue() {
            return (float) doubleValue();
        }

        @Override
        public int intValue() {
            return (int) doubleValue();
        }

        @Override
        public long longValue() {
            return (long) doubleValue();
        }
    }

    protected Arm() {
        // These are the setpoints for the joints relative to the "horizontal" (like the
        // unit circle) -- keep both
        shoulderTargetDegrees = new SmartNumber("Arm/Shoulder/Target Angle (deg)", -90);
        wristTargetDegrees = new SmartNumber("Arm/Wrist/Target Angle (deg)", +90);

        // These numbers are used for disabling/enabling wrist control while the
        // shoulder is moving -- they are no longer necessary. Remove them
        shoulderVelocityFeedbackDebounce = new SmartNumber(
                "Arm/Wrist/Feedback Enabled Debounce",
                Wrist.TELEOP_SHOULDER_VELOCITY_FEEDBACK_DEBOUNCE.doubleValue());

        shoulderVelocityFeedbackCutoff = new SmartNumber(
                "Arm/Wrist/Shoulder Velocity Feedback Cutoff (deg per s)",
                Wrist.TELEOP_SHOULDER_VELOCITY_FEEDBACK_CUTOFF.doubleValue());

        shoulderMaxVelocity = new SmartNumber(
                "Arm/Shoulder/Max Velocity",
                Shoulder.TELEOP_MAX_VELOCITY.doubleValue());
        shoulderMaxAcceleration = new SmartNumber(
                "Arm/Shoulder/Max Acceleration",
                Shoulder.TELEOP_MAX_ACCELERATION.doubleValue());

        wristMaxVelocity = new SmartNumber(
                "Arm/Wrist/Max Velocity",
                Wrist.TELEOP_MAX_VELOCITY.doubleValue());
        wristMaxAcceleration = new SmartNumber(
                "Arm/Wrist/Max Acceleration",
                Wrist.TELEOP_MAX_ACCELERATION.doubleValue());

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV,
                Shoulder.Feedforward.kA).position()
                .add(new ArmEncoderFeedforward(new GamePiecekG()))
                .add(new ArmDriveFeedforward(new GamePiecekG(), SwerveDrive.getInstance()::getForwardAccelerationGs))
                .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
                .setSetpointFilter(
                        new MotionProfile(
                                shoulderMaxVelocity.filtered(Math::toRadians).number(),
                                shoulderMaxAcceleration.filtered(Math::toRadians).number()));

        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
                .add(new ArmEncoderAngleFeedforward(Wrist.Feedforward.kG))
                .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD)
                        .setSetpointFilter(
                                new AMotionProfile(
                                        wristMaxVelocity.filtered(Math::toRadians).number(),
                                        wristMaxAcceleration.filtered(Math::toRadians).number())));

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));

        pieceGravityCompensation = false;
    }

    //

    public void enableGamePieceGravityCompensation() {
        pieceGravityCompensation = true;
    }

    public void disableGamePieceGravityCompensation() {
        pieceGravityCompensation = false;
    }

    // Arm Control Overrides
    private final boolean isWristFeedbackEnabled() {
        return Math.abs(getShoulderVelocityRadiansPerSecond()) < Units
                .degreesToRadians(shoulderVelocityFeedbackCutoff.doubleValue());
    }

    // Set kinematic constraints

    public final void setShoulderConstraints(Number velocity, Number acceleration) {
        shoulderMaxVelocity.set(velocity);
        shoulderMaxAcceleration.set(acceleration);
    }

    public final void setWristConstraints(Number velocity, Number acceleration) {
        wristMaxVelocity.set(velocity);
        wristMaxAcceleration.set(acceleration);
    }

    // Read target State
    public final Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(shoulderTargetDegrees.get());
    }

    public final Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetDegrees.get());
    }

    public final ArmState getTargetState() {
        return new ArmState(getShoulderTargetAngle(), getWristTargetAngle());
    }

    public final void setShoulderVelocityFeedbackDebounce(double time) {
        shoulderVelocityFeedbackDebounce.set(time);
    }

    public final void setShoulderVelocityFeedbackCutoff(double velocity) {
        shoulderVelocityFeedbackCutoff.set(velocity);
    }

    // Set target states
    private static double getWrappedShoulderAngle(Rotation2d angle) {
        return MathUtil.inputModulus(angle.getRadians(), Units.degreesToRadians(-270), Units.degreesToRadians(+90));
    }

    public final void setShoulderTargetAngle(Rotation2d angle) {
        shoulderTargetDegrees.set(angle.getDegrees());
    }

    public final void setWristTargetAngle(Rotation2d angle) {
        wristTargetDegrees.set(angle.getDegrees());
    }

    public final void setTargetState(ArmState state) {
        setShoulderTargetAngle(state.getShoulderState());
        setWristTargetAngle(state.getWristState());
    }

    // Change target state
    public final void moveShoulderTargetAngle(double deltaDegrees) {
        setShoulderTargetAngle(getShoulderTargetAngle().plus(Rotation2d.fromDegrees(deltaDegrees)));
    }

    public final void moveWristTargetAngle(double deltaDegrees) {
        setWristTargetAngle(getWristTargetAngle().plus(Rotation2d.fromDegrees(deltaDegrees)));
    }

    // Check if at target
    public final boolean isShoulderAtTarget(double epsilonDegrees) {
        return Math.abs(getShoulderTargetAngle().minus(getShoulderAngle()).getDegrees()) < epsilonDegrees;
    }

    public final boolean isWristAtTarget(double epsilonDegrees) {
        return Math.abs(getWristTargetAngle().minus(getWristAngle()).getDegrees()) < epsilonDegrees;
    }

    public final boolean isAtTargetState(double shoulderEpsilonDegrees, double wristEpsilonDegrees) {
        return isShoulderAtTarget(shoulderEpsilonDegrees) && isWristAtTarget(wristEpsilonDegrees);
    }

    // Read angle measurements
    public abstract Rotation2d getShoulderAngle();

    protected abstract Rotation2d getRelativeWristAngle();

    public final Rotation2d getWristAngle() {
        return getShoulderAngle().plus(getRelativeWristAngle());
    }

    public final ArmState getState() {
        return new ArmState(getShoulderAngle(), getWristAngle());
    }

    public abstract double getShoulderVelocityRadiansPerSecond();

    public abstract double getWristVelocityRadiansPerSecond();

    // set coast / brake mode
    public abstract void setCoast(boolean wristCoast, boolean shoulderCoast);

    // Arm Visualizer
    public final ArmVisualizer getVisualizer() {
        return armVisualizer;
    

    @Override
    public final void periodic() {
        // Validate shoulder and wrist target states
        Rotation2d shoulderTarget = getShoulderTargetAngle();
        // Rotation2d wristTarget = getWristTargetAngle();

        double normalizedDeg = shoulderTarget.minus(Rotation2d.fromDegrees(-90)).getDegrees();

        if (normalizedDeg > Shoulder.MAX_SHOULDER_ANGLE.get() + 90) {
            setShoulderTargetAngle(Rotation2d.fromDegrees(Shoulder.MAX_SHOULDER_ANGLE.get()));
        } else if (normalizedDeg < -90 - Shoulder.MAX_SHOULDER_ANGLE.get()) {
            setShoulderTargetAngle(Rotation2d.fromDegrees(180 - Shoulder.MAX_SHOULDER_ANGLE.get()));
        }

        // Run control loops on validated target angles
        shoulderController.update(
                getWrappedShoulderAngle(getShoulderTargetAngle()),
                getWrappedShoulderAngle(getShoulderAngle()));

        SmartDashboard.putNumber("Arm/Shoulder/Wrapped Angle", getWrappedShoulderAngle(getShoulderAngle()));
        SmartDashboard.putNumber("Arm/Shoulder/Wrapped Target Angle",
                getWrappedShoulderAngle(getShoulderTargetAngle()));

        wristController.update(
                Angle.fromRotation2d(getWristTargetAngle()),
                Angle.fromRotation2d(getWristAngle()));

        armVisualizer.setTargetAngles(Units.radiansToDegrees(shoulderController.getSetpoint()),
                wristController.getSetpoint().toDegrees());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)",
                Units.radiansToDegrees(shoulderController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", Units.radiansToDegrees(shoulderController.getError()));
        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Shoulder/Velocity (deg per s)",
                Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Relative Angle (deg)", getRelativeWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Setpoint (deg)", wristController.getSetpoint().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", wristController.getError().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist/Velocity (deg per s)",
                Units.radiansToDegrees(getWristVelocityRadiansPerSecond()));
        SmartDashboard.putBoolean("Arm/Wrist/Feedback Enabled Raw", isWristFeedbackEnabled());
        SmartDashboard.putNumber("Arm/Shoulder/kG", new GamePiecekG().doubleValue());

        SmartDashboard.putBoolean("Arm/Shoulder/Game Piece Compensation", pieceGravityCompensation);

        periodicallyCalled();
    }

    public void periodicallyCalled() {
    }
}
