package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.filters.MotionProfile;
import com.stuypulse.robot.util.AngleVelocity;
import com.stuypulse.robot.util.ArmDriveFeedforward;
import com.stuypulse.robot.util.ArmEncoderAngleFeedforward;
import com.stuypulse.robot.util.ArmEncoderFeedforward;

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
  * - setpoint control (PID+FF controllers are used) (shoulder is not allowed above maximum shoulder angle)
  * - limp mode (controller output is overriden to be zero)
  * - voltage override ("force" feeds a voltage to the motor)
  */ 
public abstract class Arm extends SubsystemBase {

    // Singleton
    private static final Arm instance;

    static {
        if (RobotBase.isSimulation())
            instance = new PerfectArm();
            // instance = new SimArm();
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
            .setOutputFilter(x -> wristVoltageOverride.orElse(isWristFeedbackEnabled() ? x : 0))
        ;

        wristVoltageOverride = Optional.of(0.0);
        shoulderVoltageOverride = Optional.of(0.0);

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));
    }

    // Arm Control Overrides

    private final boolean isWristFeedbackEnabled() {
        final double velocity = Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond());
        return Math.abs(velocity) < Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.get();
    }

    // Read target State
    public final double getShoulderTargetRadians() {
        return Math.toRadians(shoulderTargetDegrees.get());
    }
    
    public final double getWristTargetRadians() {
        return Math.toRadians(wristTargetDegrees.get());
    }

    public final ArmState getTargetState() {
        return new ArmState( (Number) Math.toDegrees(getShoulderTargetRadians()) , (Number) Math.toDegrees(getWristTargetRadians()) );
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
        setShoulderTargetAngle (Rotation2d.fromDegrees(Math.toDegrees(getShoulderTargetRadians()) + deltaDegrees));
    }

    public final void moveWristTargetAngle(double deltaDegrees) {

        setWristTargetAngle ( Rotation2d.fromDegrees(Math.toDegrees(getWristTargetRadians()) + deltaDegrees));
    }

    // Check if at target
    public final boolean isShoulderAtTarget(double epsilonDegrees) {
        return Math.abs(getShoulderTargetRadians() - getShoulderRadians()) < epsilonDegrees;
    }

    public final boolean isWristAtTarget(double epsilonDegrees) {
        return Math.abs(getWristTargetRadians() - getWristRadians()) < epsilonDegrees;
    }

    public final boolean isAtTargetState(double shoulderEpsilonDegrees, double wristEpsilonDegrees) {
        return isShoulderAtTarget(shoulderEpsilonDegrees) && isWristAtTarget(wristEpsilonDegrees);
    }

    // Read angle measurements
    public abstract double getShoulderRadians();
    protected abstract double getRelativeWristRadians();

    public final double getWristRadians() {
        return getShoulderRadians() + getRelativeWristRadians();
    }
 
    public final ArmState getState() {
        return new ArmState(getShoulderRadians(), getWristRadians());
    }

    public abstract Double getShoulderVelocityRadiansPerSecond();
    public abstract Double getWristVelocityRadiansPerSecond();

    // Set a voltage override
    public void setShoulderVoltage(Double voltage) {
        shoulderVoltageOverride = Optional.of(voltage);
    }

    public void setWristVoltage(Double voltage) {
        wristVoltageOverride = Optional.of(voltage);
    }

    // Feed a voltage to the hardware layer
    protected abstract void setShoulderVoltageImpl(Double voltage);
    protected abstract void setWristVoltageImpl(Double voltage);

    // set coast / brake mode
    public void setWristIdleMode(IdleMode mode) {}

    public final void enableShoulderBrakeMode() {}
    public final void disableShoulderBrakeMode() {}

    public void enableWristBrakeMode() {}
    public void disableWristBrakeMode() {}


    public void setCoast(boolean wristCoast, boolean shoulderCoast) {}

    // Arm Visualizer
    public final ArmVisualizer getVisualizer() {
        return armVisualizer;
    }

    @Override
    public final void periodic() {
        // Validate shoulder and wrist target states
        double shoulderTarget = getShoulderTargetRadians();

        double normalizedDeg = 0;

        if (shoulderTarget < 0) {
            normalizedDeg = shoulderTarget + (2*Math.PI);
        }
        else {
            normalizedDeg = shoulderTarget;
        } 
            

        if (normalizedDeg > (Math.toRadians(Shoulder.MAX_SHOULDER_ANGLE.get() + 90))) {
            setShoulderTargetAngle(Rotation2d.fromRadians(normalizedDeg));
        } else if (normalizedDeg < (-90 - Shoulder.MAX_SHOULDER_ANGLE.get()) * 180 / Math.PI) {
            setShoulderTargetAngle(Rotation2d.fromDegrees(180 - Shoulder.MAX_SHOULDER_ANGLE.get()));
        }


        // Run control loops on validated target angles
        shoulderController.update(
            getShoulderTargetRadians(), 
            getShoulderRadians());

        wristController.update(
            getWristTargetRadians(), 
            getWristRadians());

        setWristVoltageImpl(wristController.getOutput());
        setShoulderVoltageImpl(shoulderController.getOutput());
        

        armVisualizer.setTargetAngles(Math.toDegrees(shoulderController.getSetpoint()), Math.toDegrees(wristController.getSetpoint()));
        armVisualizer.setMeasuredAngles(Math.toDegrees(getShoulderRadians()), Math.toDegrees(getWristRadians()));
        armVisualizer.setFieldArm(Odometry.getInstance().getPose(), getState());
        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg) ", Math.toDegrees(getShoulderRadians()));
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)",
        
        
        
         Math.toDegrees(shoulderController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", Math.toDegrees(shoulderController.getError()));
        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Shoulder/Velocity (deg per s)", Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", Math.toDegrees(getWristRadians()));
        SmartDashboard.putNumber("Arm/Wrist/Relative Angle (deg)", Math.toDegrees(getRelativeWristRadians()));
        SmartDashboard.putNumber("Arm/Wrist/Setpoint (deg)", Math.toDegrees(wristController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", Math.toDegrees(wristController.getError()));
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist/Velocity (deg per s)", Units.radiansToDegrees(getWristVelocityRadiansPerSecond()));
        SmartDashboard.putBoolean("Arm/Wrist/Feedback Enabled", isWristFeedbackEnabled());

        periodicallyCalled();
    }

    public void periodicallyCalled() {}
}
//call set target angle for the wrist (-90 to 270) for the shoulder (90, -270)
//take underlined degree value and make sure its in the right rannge, make sure sure that it is in radians
//setTargetSomething , keep it in degrees but if we are getSomething, convert that from degrees into radians

/** 
enableShoulderBrakeMode
disableShoulderBrakeMode
enableWristBrakeMode
disableWristBrakeMode 
*/