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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

        SmartDashboard.putData(instance);
    }

    public static Arm getInstance() {
        return instance;
    }

    /* ARM VARIABLES */

    // represents a goal for the arm (separate from the profiled setpoints)
    private double shoulderTargetDegrees;
    private double wristTargetDegrees;

    // shoulder controllers
    private final SimpleMotorFeedforward shoulderFF;
    private final ProfiledPIDController shoulderProfilePID;
    private final GamePiecekG shoulderkG;

    // wrist controllers
    private final SimpleMotorFeedforward wristFF;
    private final ProfiledPIDController wristProfilePID;

    // Mechanism2d visualizer
    private final ArmVisualizer armVisualizer;

    // Limp mode (forces a joint to receive zero voltage)
    private boolean wristLimp;
    private boolean shoulderLimp;

    // Voltage overrides (used when present)
    private Optional<Double> wristVoltageOverride;
    private Optional<Double> shoulderVoltageOverride;

    private BStream wristEnabled;

    private double shoulderVelocityFeedbackDebounce;
    private double shoulderVelocityFeedbackCutoff;

    private boolean pieceGravityCompensation;

    private double shoulderMaxVelocity;
    private double shoulderMaxAcceleration;

    private double wristMaxVelocity;
    private double wristMaxAcceleration;

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

            builder.addDoubleProperty("kGEmpty", () -> kGEmpty, (double kG) -> kGEmpty = kG );
            builder.addDoubleProperty("kGCube", () -> kGCube, (double kG) -> kGCube = kG );
            builder.addDoubleProperty("kGCone", () -> kGCone, (double kG) -> kGCone = kG );
        }
    }

    protected Arm() {
        shoulderTargetDegrees = -90;
        wristTargetDegrees = +90;

        shoulderVelocityFeedbackDebounce = Wrist.TELEOP_SHOULDER_VELOCITY_FEEDBACK_DEBOUNCE;
        shoulderVelocityFeedbackCutoff = Wrist.TELEOP_SHOULDER_VELOCITY_FEEDBACK_CUTOFF;

        wristEnabled = BStream.create(this::isWristFeedbackEnabled)
            .filtered(new BDebounce.Both(shoulderVelocityFeedbackDebounce.get()));

        shoulderMaxVelocity = Shoulder.TELEOP_MAX_VELOCITY;
        shoulderMaxAcceleration = Shoulder.TELEOP_MAX_ACCELERATION;

        wristMaxVelocity = Wrist.TELEOP_MAX_VELOCITY;
        wristMaxAcceleration = Wrist.TELEOP_MAX_ACCELERATION;

        shoulderkG = new GamePiecekG(Shoulder.Feedforward.kGEmpty, Shoulder.Feedforward.kGCube, Shoulder.Feedforward.kGCone);
        shoulderFF = new SimpleMotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA);
        shoulderProfilePID = new ProfiledPIDController(
            Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD,
            new Constraints(
                Math.toRadians(shoulderMaxVelocity),
                Math.toRadians(shoulderMaxAcceleration)));

        shoulderProfilePID.enableContinuousInput(-Math.PI, +Math.PI);

        wristFF = new SimpleMotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA);
        wristProfilePID = new ProfiledPIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD,
            new Constraints(
                Math.toRadians(wristMaxVelocity),
                Math.toRadians(wristMaxAcceleration)));
        
        wristProfilePID.enableContinuousInput(-Math.PI, +Math.PI);

        wristLimp = false;
        shoulderLimp = false;

        wristVoltageOverride = Optional.empty();
        shoulderVoltageOverride = Optional.empty();

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));

        pieceGravityCompensation = false;
    }

    public void resetMotionProfile() {
        shoulderProfilePID.reset(getShoulderAngle().getRadians());
    }

    // Arm Control

    private double calculateShoulder() {
        // goes first to update motion profile
        double output = shoulderProfilePID.calculate(getShoulderAngle().getRadians());

        State setpoint = shoulderProfilePID.getSetpoint();

        output += shoulderFF.calculate(setpoint.velocity)
            + shoulderkG.calculate(getShoulderAngle().getRadians());

        if (isShoulderLimp()) return 0;

        return shoulderVoltageOverride.orElse(output);
    }

    private double calculateWrist() {
        // goes first to update motion profile
        double output = wristProfilePID.calculate(getWristAngle().getRadians());

        if (!wristEnabled.get()) output = 0;

        output += wristFF.calculate(wristProfilePID.getSetpoint().velocity)
            + getWristAngle().getCos() * Wrist.Feedforward.kG;

        if (isWristLimp()) return 0;

        return wristVoltageOverride.orElse(output);
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
        return wristLimp;
    }

    private final boolean isShoulderLimp() {
        return shoulderLimp;
    }

    private final boolean isWristFeedbackEnabled() {
        return Math.abs(getShoulderVelocityRadiansPerSecond()) < Units.degreesToRadians(shoulderVelocityFeedbackCutoff);
    }

    // Set kinematic constraints

    public final void setShoulderConstraints(double velocity, double acceleration) {
        shoulderMaxVelocity = velocity;
        shoulderMaxAcceleration = acceleration;
    }

    public final void setWristConstraints(double velocity, double acceleration) {
        wristMaxVelocity = velocity;
        wristMaxAcceleration = acceleration;
    }


    // Read target State
    public final Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(shoulderTargetDegrees);
    }

    public final Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetDegrees);
    }

    public final ArmState getTargetState() {
        return new ArmState(getShoulderTargetAngle(), getWristTargetAngle());
    }

    public final void setShoulderVelocityFeedbackDebounce(double time) {
        shoulderVelocityFeedbackDebounce = time;
    }

    public final void setShoulderVelocityFeedbackCutoff(double velocity) {
        shoulderVelocityFeedbackCutoff = velocity;
    }

    // Set target states
    private static double getWrappedShoulderAngle(Rotation2d angle) {
        return MathUtil.inputModulus(angle.getRadians(), Units.degreesToRadians(-270), Units.degreesToRadians(+90));
    }

    public final void setShoulderTargetAngle(Rotation2d angle) {
        shoulderVoltageOverride = Optional.empty();
        shoulderTargetDegrees = angle.getDegrees();

        shoulderProfilePID.reset(getShoulderAngle().getRadians());
        shoulderProfilePID.setGoal(angle.getRadians());
    }

    public final void setWristTargetAngle(Rotation2d angle) {
        wristVoltageOverride = Optional.empty();
        wristTargetDegrees = angle.getDegrees();
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
        this.wristLimp = wristLimp;
        this.shoulderLimp = shoulderLimp;
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

        double wristVoltage = calculateWrist();

        setWristVoltageImpl(wristVoltage);
        setShoulderVoltageImpl(shoulderVoltage);

        armVisualizer.setTargetAngles(Units.radiansToDegrees(shoulderProfilePID.getSetpoint().position), Units.radiansToDegrees(wristProfilePID.getSetpoint().position));
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putData("Arm/Shoulder/PID", shoulderProfilePID);
        SmartDashboard.putData("Arm/Shoulder/Feedforward", shoulderFF);
        SmartDashboard.putData("Arm/Shoulder/GamePiecekG", shoulderkG);

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)", Units.radiansToDegrees(shoulderProfilePID.getSetpoint().position));
        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderVoltage);
        SmartDashboard.putNumber("Arm/Shoulder/Velocity (deg per s)", Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Relative Angle (deg)", getRelativeWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Setpoint (deg)", Units.radiansToDegrees(wristProfilePID.getSetpoint().position));
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristVoltage);
        SmartDashboard.putNumber("Arm/Wrist/Velocity (deg per s)", Units.radiansToDegrees(getWristVelocityRadiansPerSecond()));
        SmartDashboard.putBoolean("Arm/Wrist/Feedback Enabled Raw", isWristFeedbackEnabled());
        SmartDashboard.putBoolean("Arm/Wrist/Feedback Enabled", wristEnabled.get());
        SmartDashboard.putNumber("Arm/Shoulder/kG", shoulderkG.getkG());

        SmartDashboard.putBoolean("Arm/Shoulder/Game Piece Compensation", pieceGravityCompensation);

        periodicallyCalled();
    }

    public void periodicallyCalled() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Shoulder Target Degrees", () -> shoulderTargetDegrees, (double deg) -> shoulderTargetDegrees = deg);
        builder.addDoubleProperty("Wrist Target Degrees", () -> wristTargetDegrees, (double deg) -> wristTargetDegrees = deg);

        builder.addBooleanProperty("Wrist Limp", () -> wristLimp, (boolean limp) -> wristLimp = limp);
        builder.addBooleanProperty("Shoulder Limp", () -> shoulderLimp, (boolean limp) -> shoulderLimp = limp);

        builder.addDoubleProperty("Shoulder Max Velocity", () -> shoulderMaxVelocity, (double vel) -> shoulderMaxVelocity = vel);
        builder.addDoubleProperty("Wrist Max Velocity", () -> wristMaxVelocity, (double vel) -> wristMaxVelocity = vel);
        builder.addDoubleProperty("Shoulder Max Acceleration", () -> shoulderMaxAcceleration, (double accel) -> shoulderMaxAcceleration = accel);
        builder.addDoubleProperty("Wrist Max Acceleration", () -> wristMaxAcceleration, (double accel) -> wristMaxAcceleration = accel);
        
        builder.addDoubleProperty("Shoulder Velocity Feedback Debounce", () -> shoulderVelocityFeedbackDebounce, (double debounce) -> shoulderVelocityFeedbackDebounce = debounce);
        builder.addDoubleProperty("Shoulder Velocity Feedback Cutoff", () -> shoulderVelocityFeedbackCutoff, (double vel) -> shoulderVelocityFeedbackCutoff = vel);
    }
}
