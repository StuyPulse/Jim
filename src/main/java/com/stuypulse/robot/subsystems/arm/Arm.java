package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.robot.util.AngleVelocity;
import com.stuypulse.robot.util.ArmEncoderAngleFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

    
/**
  * Manual control
  * 1. schedule setpoints
  * 2. joystick
  * 3. cancel commands, set target state to current state
  * 4. move around
  * 5. schedule setpoints again
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
        // instance = new SimArm();
    }

    public static Arm getInstance() {
        return instance;
    }

    /* ARM VARIABLES */

    // represents a goal for the arm (separate from the profiled setpoints)
    private final SmartNumber shoulderTargetDegrees;
    private final SmartNumber wristTargetDegrees;

    private final SmartNumber shoulderMaxVelocity;
    private final SmartNumber shoulderMaxAcceleration;

    private final SmartNumber wristMaxVelocity;
    private final SmartNumber wristMaxAcceleration;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final AngleVelocity shoulderVelocity;

    private final ArmVisualizer armVisualizer;

    private boolean wristLimp;
    private boolean shoulderLimp;

    private Optional<Double> wristVoltageOverride;
    private Optional<Double> shoulderVoltageOverride;

    public Arm() {
        shoulderTargetDegrees = new SmartNumber("Arm/Shoulder/Target Angle (deg)", -90);
        wristTargetDegrees = new SmartNumber("Arm/Wrist/Target Angle (deg)", +90);

        shoulderVelocity = new AngleVelocity();

        shoulderMaxVelocity = new SmartNumber("Arm/Shoulder/Max Velocity (deg per s)", 270);
        shoulderMaxAcceleration = new SmartNumber("Arm/Shoulder/Max Acceleration (deg per s^2)", 270);
        
        wristMaxVelocity = new SmartNumber("Arm/Wrist/Max Velocity (deg per s)", 360);
        wristMaxAcceleration = new SmartNumber("Arm/Wrist/Max Acceleration (deg per s^2)", 360);

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
            .add(new ArmEncoderAngleFeedforward(Shoulder.Feedforward.kG))
            .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
            .setOutputFilter(x -> {
                if (shoulderLimp) return 0;

                return x;
            })
            .setSetpointFilter(
                new AMotionProfile(
                    shoulderMaxVelocity.filtered(Math::toRadians).number(), 
                    shoulderMaxAcceleration.filtered(Math::toRadians).number())
            );
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
            .add(new ArmEncoderAngleFeedforward(Wrist.Feedforward.kG))
            .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD)
            .setOutputFilter(x -> {
                if (!isWristFeedbackEnabled()) return 0;
                if (wristLimp) return 0;

                return x;
            }))
            .setSetpointFilter(
                new AMotionProfile(
                    wristMaxVelocity.filtered(Math::toRadians).number(), 
                    wristMaxAcceleration.filtered(Math::toRadians).number())
            );

        wristLimp = false;
        shoulderLimp = false;

        wristVoltageOverride = Optional.empty();
        shoulderVoltageOverride = Optional.empty();

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));
    }

    protected boolean isWristFeedbackEnabled() {
        return Math.abs(Units.radiansToDegrees(shoulderVelocity.getOutput())) < Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.get();
    }

    // Target State
    public final Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(shoulderTargetDegrees.get());
    }
    
    public Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetDegrees.get());
    }

    public ArmState getTargetState() {
        return new ArmState(getShoulderTargetAngle(), getWristTargetAngle());
    }

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

    // Move Target State
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

    // Get angle measurements

    public abstract Rotation2d getShoulderAngle();
    protected abstract Rotation2d getRelativeWristAngle();

    public final Rotation2d getWristAngle() {
        return getShoulderAngle().plus(getRelativeWristAngle());
    }

    public final ArmState getState() {
        return new ArmState(getShoulderAngle(), getWristAngle());
    }

    public void setShoulderVoltage(double voltage) {
        shoulderVoltageOverride = Optional.of(voltage);
    }

    // TODO: arm module
    protected abstract void setShoulderVoltageImpl(double voltage);


    public void setWristVoltage(double voltage) {
        wristVoltageOverride = Optional.of(voltage);
    }

    protected abstract void setWristVoltageImpl(double voltage);

    public void setCoast(boolean wristCoast, boolean shoulderCoast) {}

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

    // Set kinematic constraints
    public void setWristConstraints(double maxVelocity, double maxAcceleration) {
        wristMaxVelocity.set(maxVelocity);
        wristMaxAcceleration.set(maxAcceleration);
    }

    public void setShoulderConstraints(double maxVelocity, double maxAcceleration) {
        shoulderMaxVelocity.set(maxVelocity);
        shoulderMaxAcceleration.set(maxAcceleration);
    }

    @Override
    public final void periodic() {
        shoulderVelocity.update(Angle.fromRotation2d(getShoulderAngle()));

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

        if (wristVoltageOverride.isPresent()) {
            setWristVoltageImpl(wristVoltageOverride.get());
        } else if (wristLimp) {
            setWristVoltageImpl(0);
        } else {
            setWristVoltageImpl(wristController.getOutput());
        }

        if (shoulderVoltageOverride.isPresent()) {
            setShoulderVoltageImpl(shoulderVoltageOverride.get());
        } else if (wristLimp) {
            setShoulderVoltageImpl(0);
        } else {
            setShoulderVoltageImpl(shoulderController.getOutput());
        }

        armVisualizer.setTargetAngles(shoulderController.getSetpoint().toDegrees(), wristController.getSetpoint().toDegrees());
        armVisualizer.setMeasuredAngles(getShoulderAngle().getDegrees(), getWristAngle().getDegrees());
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());

        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)", shoulderController.getSetpoint().toDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", shoulderController.getError().toDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Shoulder/Velocity (deg per s)", Units.radiansToDegrees(shoulderVelocity.getOutput()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Relative Angle (deg)", getRelativeWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Setpoint (deg)", wristController.getSetpoint().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", wristController.getError().toDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristController.getOutput());
        SmartDashboard.putBoolean("Arm/Wrist/Feedback Enabled", isWristFeedbackEnabled());

        periodicallyCalled();
    }

    public void periodicallyCalled() {}
}
