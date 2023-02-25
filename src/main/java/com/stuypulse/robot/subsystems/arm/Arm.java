package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.robot.util.AngleVelocity;
import com.stuypulse.robot.util.ArmConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

    private final ArmConstraints constraints;

    private final AngleController shoulderController;
    private final AngleController wristController;

    private final AngleVelocity shoulderVelocity;

    private final ArmVisualizer armVisualizer;

    private boolean limp;

    public Arm() {
        shoulderTargetDegrees = new SmartNumber("Arm/Shoulder/Target Angle (deg)", -90);
        wristTargetDegrees = new SmartNumber("Arm/Wrist/Target Angle (deg)", +90);

        constraints = new ArmConstraints(Shoulder.MAX_VELOCITY.get(), 
                                        Shoulder.MAX_ACCELERATION.get(), 
                                        Wrist.MAX_VELOCITY.get(), 
                                        Wrist.MAX_ACCELERATION.get(),
                                        Shoulder.AUTO_MAX_VELOCITY.get(),
                                        Shoulder.AUTO_MAX_ACCELERATION.get(),
                                        Wrist.AUTO_MAX_VELOCITY.get(),
                                        Wrist.AUTO_MAX_ACCELERATION.get());

        shoulderVelocity = new AngleVelocity();

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).angle()
            .add(new AngleArmFeedforward(Shoulder.Feedforward.kG))
            .add(new AnglePIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
            .setSetpointFilter(
                new AMotionProfile(
                    constraints.getShoulderMaxVelocity().number(), 
                    constraints.getShoulderMaxAcceleration().number()));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).angle()
            .add(new AngleArmFeedforward(Wrist.Feedforward.kG))
            .add(new AnglePIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD)
                .setOutputFilter(x -> isWristFeedbackEnabled() ? x : 0))
            .setSetpointFilter(
                new AMotionProfile(
                    constraints.getWristMaxVelocity().number(), 
                    constraints.getWristMaxAcceleration().number()));

        limp = false;

        armVisualizer = new ArmVisualizer(Odometry.getInstance().getField().getObject("Field Arm"));
    }

    // add withConstraints to trajectories

    public final void setConstraints(double shoulderMaxVel, double shoulderMaxAccel, double wristMaxVel, double wristMaxAccel) {
        this.constraints.update(shoulderMaxVel, shoulderMaxAccel, wristMaxVel, wristMaxAccel);
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
        shoulderTargetDegrees.set(angle.getDegrees());
    }

    public final void setWristTargetAngle(Rotation2d angle) {
        wristTargetDegrees.set(angle.getDegrees());
    }

    public final void setTargetState(ArmState state) {
        setShoulderTargetAngle(state.getShoulderState());
        setWristTargetAngle(state.getWristState());
    }

    // Move Target State
    public final void moveShoulderTargetAngle(double deltaDegrees) {
        shoulderTargetDegrees.set(shoulderTargetDegrees.get() + deltaDegrees);
    }

    public final void moveWristTargetAngle(double deltaDegrees) {
        wristTargetDegrees.set(wristTargetDegrees.get() + deltaDegrees);
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

    protected abstract void setShoulderVoltage(double voltage);
    protected abstract void setWristVoltage(double voltage);

    public void setCoast(boolean coast) {}

    public final void setLimp(boolean limp) {
        this.limp = limp;
    }
    
    public final void enableLimp() {
        setLimp(true);
    }
    public final void disableLimp() {
        setLimp(false);
    }
    
    // Arm Visualizer
    public final ArmVisualizer getVisualizer() {
        return armVisualizer;
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

        if (limp) {
            setShoulderVoltage(0);
            setWristVoltage(0);
        } else {
            setShoulderVoltage(shoulderController.getOutput());
            setWristVoltage(wristController.getOutput());
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
