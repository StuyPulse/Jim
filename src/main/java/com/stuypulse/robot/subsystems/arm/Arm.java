package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.util.AngleVelocity;
import com.stuypulse.robot.util.ArmDriveFeedforward;
import com.stuypulse.robot.util.ArmEncoderAngleFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

    
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
        if (RobotBase.isSimulation())
            instance = new SimArm();
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
    private final AngleController shoulderController;
    private final AngleController wristController;

    // Mechanism2d visualizer
    private final ArmVisualizer armVisualizer;

    // Limp mode (forces a joint to receive zero voltage)
    private SmartBoolean wristLimp;
    private SmartBoolean shoulderLimp;

    // Voltage overrides (used when present)
    private Optional<Double> wristVoltageOverride;
    private Optional<Double> shoulderVoltageOverride;

    private BStream wristEnabled;

    private SmartNumber feedbackDebounce = new SmartNumber("Arm/Wrist/Feedback Enabled Debounce", 0.1);

    protected Arm() {
        shoulderTargetDegrees = new SmartNumber("Arm/Shoulder/Target Angle (deg)", -90);
        wristTargetDegrees = new SmartNumber("Arm/Wrist/Target Angle (deg)", +90);

        wristEnabled = BStream.create(this::isWristFeedbackEnabled)
            .filtered(new BDebounce.Both(feedbackDebounce));

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
            .add(new ArmEncoderAngleFeedforward(Shoulder.Feedforward.kG))
            .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
            .setSetpointFilter(
                new AMotionProfile(
                    Shoulder.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                    Shoulder.MAX_ACCELERATION.filtered(Math::toRadians).number()))
            .setOutputFilter(x -> {
                if (isShoulderLimp()) return 0;
                return shoulderVoltageOverride.orElse(x);
            })
        ;
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
            .add(new ArmEncoderAngleFeedforward(Wrist.Feedforward.kG))
            .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD)
                .setOutputFilter(x -> wristEnabled.get() ? x : 0))
            .setSetpointFilter(
                new AMotionProfile(
                    Wrist.MAX_VELOCITY.filtered(Math::toRadians).number(), 
                    Wrist.MAX_ACCELERATION.filtered(Math::toRadians).number()))
            .setOutputFilter(x -> {
                // return wristVoltageOverride.orElse(x);
                if (isWristLimp()) return 0;

                if (wristVoltageOverride.isPresent()) return wristVoltageOverride.get();
                if (!wristEnabled.get()) return 0;
                return x;
            });

        wristLimp = new SmartBoolean("Arm/Wrist/Is Limp?", false);
        shoulderLimp = new SmartBoolean("Arm/Shoulder/Is Limp?", false);

        wristVoltageOverride = Optional.empty();
        shoulderVoltageOverride = Optional.empty();

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));
    }

    // Arm Control Overrides

    private final boolean isWristLimp() {
        return wristLimp.get();
    }

    private final boolean isShoulderLimp() {
        return shoulderLimp.get();
    }

    SmartNumber tolerance = new SmartNumber("Arm/Wrist/Feedback Tolerance (deg)", 5);
    private final boolean isWristFeedbackEnabled() {
        return getShoulderVelocityRadiansPerSecond() < Units.degreesToRadians(Settings.Arm.Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.get());
        // return isShoulderAtTarget(tolerance.doubleValue());
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

    // Set target states
    public final void setShoulderTargetAngle(Rotation2d angle) {
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
        shoulderController.update(
            Angle.fromRotation2d(getShoulderTargetAngle()), 
            Angle.fromRotation2d(getShoulderAngle()));

        wristController.update(
            Angle.fromRotation2d(getWristTargetAngle()), 
            Angle.fromRotation2d(getWristAngle()));

        setWristVoltageImpl(wristController.getOutput());
        setShoulderVoltageImpl(shoulderController.getOutput());

        armVisualizer.setTargetAngles(shoulderController.getSetpoint().toDegrees(), wristController.getSetpoint().toDegrees());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)", shoulderController.getSetpoint().toDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", shoulderController.getError().toDegrees());
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

        periodicallyCalled();
    }

    public void periodicallyCalled() {}
}
