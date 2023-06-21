/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.robot.util.BenMotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;


/**
  * Double jointed arm controlled by two motion profiled PID controllers.
  *
  * Available control "modes":
  * - setpoint control (PID+FF controllers are used) (shoulder is not allowed above maximum shoulder angle)
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

    // shoulder controllers
    private final SimpleMotorFeedforward shoulderFF;
    private final PIDController shoulderPID;
    private final GamePiecekG shoulderkG;
    private TrapezoidProfile shoulderMotionProfile;
    private final Timer shoulderProfileTimer;

    // wrist controllers
    private final SimpleMotorFeedforward wristFF;
    private final PIDController wristPID;
    // private final TrapezoidProfile wristMotionProfile;
    // private final Timer wristProfileTimer;

    // Mechanism2d visualizer
    private final ArmVisualizer armVisualizer;

    // Limp mode (forces a joint to receive zero voltage)
    private SmartBoolean wristLimp;
    private SmartBoolean shoulderLimp;

    // Voltage overrides (used when present)
    private Optional<Double> wristVoltageOverride;
    private Optional<Double> shoulderVoltageOverride;

    private BStream wristEnabled;

    private SmartNumber shoulderVelocityFeedbackDebounce;
    private SmartNumber shoulderVelocityFeedbackCutoff;

    private boolean pieceGravityCompensation;

    private final SmartNumber shoulderMaxVelocity;
    private final SmartNumber shoulderMaxAcceleration;

    private final SmartNumber wristMaxVelocity;
    private final SmartNumber wristMaxAcceleration;

    private class GamePiecekG implements Sendable {
        private double kGEmpty;
        private double kGCube;
        private double kGCone;
    
        public double calculate(double positionRadians) {
            return getkG() * Math.cos(positionRadians);
        }
    
        public double getkG() {
            if (!pieceGravityCompensation) {
                return Shoulder.Feedforward.kGEmpty;
            }
    
            switch (Manager.getInstance().getGamePiece()) {
                case CONE_TIP_IN:
                case CONE_TIP_OUT:
                case CONE_TIP_UP:
                    return kGCone;
                case CUBE:
                    return kGCube;
                default:
                    return kGEmpty;
            }
        }
    
        public GamePiecekG(double kGEmpty, double kGCube, double kGCone) {
            this.kGEmpty = kGEmpty;
            this.kGCube = kGCube;
            this.kGCone = kGCone;
        }
    
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Gamepiece kG Switcher");

            builder.addDoubleProperty("kGEmpty", () -> kGEmpty, (double kG) -> { kGEmpty = kG; });
            builder.addDoubleProperty("kGCube", () -> kGCube, (double kG) -> { kGCube = kG; });
            builder.addDoubleProperty("kGCone", () -> kGCone, (double kG) -> { kGCone = kG; });
        }
    }

    protected Arm() {
        shoulderTargetDegrees = new SmartNumber("Arm/Shoulder/Target Angle (deg)", -90);
        wristTargetDegrees = new SmartNumber("Arm/Wrist/Target Angle (deg)", +90);

        shoulderVelocityFeedbackDebounce = new SmartNumber(
            "Arm/Wrist/Feedback Enabled Debounce",
            Wrist.TELEOP_SHOULDER_VELOCITY_FEEDBACK_DEBOUNCE.doubleValue());

        shoulderVelocityFeedbackCutoff = new SmartNumber(
            "Arm/Wrist/Shoulder Velocity Feedback Cutoff (deg per s)",
            Wrist.TELEOP_SHOULDER_VELOCITY_FEEDBACK_CUTOFF.doubleValue());

        wristEnabled = BStream.create(this::isWristFeedbackEnabled)
            .filtered(new BDebounce.Both(shoulderVelocityFeedbackDebounce));

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

        shoulderFF = new SimpleMotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA);
        shoulderPID = new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD);
        shoulderkG = new GamePiecekG(Shoulder.Feedforward.kGEmpty, Shoulder.Feedforward.kGCube, Shoulder.Feedforward.kGCone);
        shoulderMotionProfile = new TrapezoidProfile(
            new Constraints(
                Math.toRadians(shoulderMaxVelocity.get()),
                Math.toRadians(shoulderMaxAcceleration.get())),
            new State(shoulderTargetDegrees.get(), 0));

        wristFF = new SimpleMotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA);
            .add(new ArmEncoderAngleFeedforward(Wrist.Feedforward.kG))
            .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD)
                .setOutputFilter(x -> wristEnabled.get() ? x : 0))
            .setSetpointFilter(
                new AMotionProfile(
                    wristMaxVelocity.filtered(Math::toRadians).number(),
                    wristMaxAcceleration.filtered(Math::toRadians).number()))
            .setOutputFilter(x -> {
                if (isWristLimp()) return 0;
                return wristVoltageOverride.orElse(x);
            });

        wristLimp = new SmartBoolean("Arm/Wrist/Is Limp?", false);
        shoulderLimp = new SmartBoolean("Arm/Shoulder/Is Limp?", false);

        wristVoltageOverride = Optional.empty();
        shoulderVoltageOverride = Optional.empty();

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));

        pieceGravityCompensation = false;
    }

    public void resetMotionProfile() {
        shoulderMotionProfile.reset(getShoulderAngle().getRadians());
    }

    // Arm Control

    private double calculateShoulder() {
        State setpoint = shoulderMotionProfile.calculate(shoulderProfileTimer.get());

        double output = shoulderFF.calculate(setpoint.velocity)
            + shoulderPID.calculate(getShoulderAngle().getRadians(), setpoint.position)
            + shoulderkG.calculate(getShoulderAngle().getRadians());

        if (isShoulderLimp()) return 0;

        return shoulderVoltageOverride.orElse(output);
    }

    private double calculateWrist() {

    }

    //

    public void enableGamePieceGravityCompensation() {
        pieceGravityCompensation = true;
    }

    public void disableGamePieceGravityCompensation() {
        pieceGravityCompensation = false;
    }

    // Arm Control Overrides

    private final boolean isWristLimp() {
        return wristLimp.get();
    }

    private final boolean isShoulderLimp() {
        return shoulderLimp.get();
    }

    private final boolean isWristFeedbackEnabled() {
        return Math.abs(getShoulderVelocityRadiansPerSecond()) < Units.degreesToRadians(shoulderVelocityFeedbackCutoff.doubleValue());
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
        shoulderMotionProfile = new TrapezoidProfile(
            new Constraints(
                Math.toRadians(shoulderMaxVelocity.get()),
                Math.toRadians(shoulderMaxAcceleration.get())),
            new State(shoulderTargetDegrees.get(), 0),
            new State(getShoulderAngle().getRadians(), getShoulderVelocityRadiansPerSecond()));

        shoulderProfileTimer.restart();

        shoulderVoltageOverride = Optional.empty();
        shoulderTargetDegrees.set(angle.getDegrees());
    }

    public final void setWristTargetAngle(Rotation2d angle) {
        wristVoltageOverride = Optional.empty();
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

    // Set a voltage override
    public void setShoulderVoltage(double voltage) {
        shoulderVoltageOverride = Optional.of(voltage);
    }

    public void setWristVoltage(double voltage) {
        wristVoltageOverride = Optional.of(voltage);
    }

    // Feed a voltage to the hardware layer
    protected abstract void setShoulderVoltageImpl(double voltage);
    protected abstract void setWristVoltageImpl(double voltage);

    // set coast / brake mode
    public void setCoast(boolean wristCoast, boolean shoulderCoast) {}

    // set if the ligaments are "limp" (zero voltage)
    public final void setLimp(boolean wristLimp, boolean shoulderLimp) {
        this.wristLimp.set(wristLimp);
        this.shoulderLimp.set(shoulderLimp);
    }

    public final void enableLimp() {
        setLimp(true, true);
    }
    public final void disableLimp() {
        setLimp(false, false);
    }

    // Arm Visualizer
    public final ArmVisualizer getVisualizer() {
        return armVisualizer;
    }

    @Override
    public final void periodic() {
        // Validate shoulder and wrist target states
        Rotation2d shoulderTarget = getShoulderTargetAngle();
        Rotation2d wristTarget = getWristTargetAngle();

        double normalizedDeg = shoulderTarget.minus(Rotation2d.fromDegrees(-90)).getDegrees();

        if (normalizedDeg > Shoulder.MAX_SHOULDER_ANGLE.get() + 90) {
            setShoulderTargetAngle(Rotation2d.fromDegrees(Shoulder.MAX_SHOULDER_ANGLE.get()));
        } else if (normalizedDeg < -90 - Shoulder.MAX_SHOULDER_ANGLE.get()) {
            setShoulderTargetAngle(Rotation2d.fromDegrees(180 - Shoulder.MAX_SHOULDER_ANGLE.get()));
        }


        // Run control loops on validated target angles
        double shoulderVoltage = calculateShoulder();

        SmartDashboard.putNumber("Arm/Shoulder/Wrapped Angle", getWrappedShoulderAngle(getShoulderAngle()));
        SmartDashboard.putNumber("Arm/Shoulder/Wrapped Target Angle", getWrappedShoulderAngle(getShoulderTargetAngle()));

        wristController.update(
            Angle.fromRotation2d(getWristTargetAngle()),
            Angle.fromRotation2d(getWristAngle()));

        setWristVoltageImpl(wristController.getOutput());
        setShoulderVoltageImpl(shoulderController.getOutput());

        armVisualizer.setTargetAngles(Units.radiansToDegrees(shoulderController.getSetpoint()), wristController.getSetpoint().toDegrees());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putData("Arm/Shoulder/PID", shoulderPID);
        SmartDashboard.putData("Arm/Shoulder/Feedforward", shoulderFF);
        SmartDashboard.putData("Arm/Shoulder/GamePiecekG", shoulderkG);

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)", Units.radiansToDegrees(shoulderController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", Units.radiansToDegrees(shoulderController.getError()));
        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Shoulder/Velocity (deg per s)", Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Relative Angle (deg)", getRelativeWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Setpoint (deg)", wristController.getSetpoint().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", wristController.getError().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist/Velocity (deg per s)", Units.radiansToDegrees(getWristVelocityRadiansPerSecond()));
        SmartDashboard.putBoolean("Arm/Wrist/Feedback Enabled Raw", isWristFeedbackEnabled());
        SmartDashboard.putBoolean("Arm/Wrist/Feedback Enabled", wristEnabled.get());
        SmartDashboard.putNumber("Arm/Shoulder/kG", shoulderkG.getkG());

        SmartDashboard.putBoolean("Arm/Shoulder/Game Piece Compensation", pieceGravityCompensation);

        periodicallyCalled();
    }

    public void periodicallyCalled() {}
}
