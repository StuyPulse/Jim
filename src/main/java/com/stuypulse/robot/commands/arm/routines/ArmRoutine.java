package com.stuypulse.robot.commands.arm.routines;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class ArmRoutine extends CommandBase {

    private Number shoulderTolerance;
    private Number wristTolerance;

    private Number shoulderVelocityTolerance;
    private Number wristVelocityTolerance;

    private Number shoulderMaxVelocity;
    private Number shoulderMaxAcceleration;

    private Number wristMaxVelocity;
    private Number wristMaxAcceleration;
    
    protected final Arm arm;
    protected final Supplier<ArmState> endState;
    
    protected ArmTrajectory trajectory;
    private int currentIndex;

    public ArmRoutine(Supplier<ArmState> endState) {
        arm = Arm.getInstance();

        shoulderTolerance = Shoulder.TOLERANCE;
        wristTolerance = Wrist.TOLERANCE;
        
        this.endState = endState;
        currentIndex = 0;

        shoulderVelocityTolerance = 1000000;
        wristVelocityTolerance = 1000000;
        
        shoulderMaxVelocity = Shoulder.MAX_VELOCITY;
        shoulderMaxAcceleration = Shoulder.MAX_ACCELERATION;

        wristMaxVelocity = Wrist.MAX_VELOCITY;
        wristMaxAcceleration = Wrist.MAX_ACCELERATION;

        addRequirements(arm);
    }

    public ArmRoutine setWristVelocityTolerance(double toleranceDegreesPerSecond) {
        wristVelocityTolerance = toleranceDegreesPerSecond;
        return this;
    }

    public ArmRoutine setShoulderVelocityTolerance(double toleranceDegreesPerSecond) {
        shoulderVelocityTolerance = toleranceDegreesPerSecond;
        return this;
    }

    protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        // TODO: check if src and dest are on the same side
        double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();

        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), wristSafeAngle)
            .addState(
                new ArmState(dest.getShoulderDegrees(), wristSafeAngle).setWristLimp(true).setWristTolerance(360))
            .addState(dest);
    }

    @Override
    public void initialize() {
        trajectory = getTrajectory(Arm.getInstance().getState(), endState.get());
        
        arm.disableGamePieceGravityCompensation();
        // for (ArmState state : trajectory.getStates()) {
        //     System.out.println("Shoulder: " + state.getShoulderDegrees() + ", Wrist: " + state.getWristDegrees());
        // }
        // System.out.println();

        arm.setShoulderConstraints(shoulderMaxVelocity, shoulderMaxAcceleration);
        arm.setWristConstraints(wristMaxVelocity, wristMaxAcceleration);
        
        currentIndex = 0;
    }

    @Override
    public void execute() {
        var targetState = trajectory.getStates().get(currentIndex);

        arm.setShoulderConstraints(
            targetState.getShoulderMaxVelocity().orElse(shoulderMaxVelocity), 
            targetState.getShoulderMaxAcceleration().orElse(shoulderMaxAcceleration));
        arm.setWristConstraints(
            targetState.getWristMaxVelocity().orElse(wristMaxVelocity), 
            targetState.getWristMaxAcceleration().orElse(wristMaxAcceleration));
        
        arm.setTargetState(targetState);

        arm.setLimp(targetState.isWristLimp(), false);

        double currentShoulderTolerance = (targetState.getShoulderTolerance().orElse(shoulderTolerance)).doubleValue();
        double currentWristTolerance = (targetState.getWristTolerance().orElse(wristTolerance)).doubleValue();

        SmartDashboard.putNumber("Arm/Shoulder/Current Tolerance (deg)", currentShoulderTolerance);
        SmartDashboard.putNumber("Arm/Wrist/Current Tolerance (deg)", currentWristTolerance);

        if (targetState.isWristLimp()) {
            currentWristTolerance = 360;
        }

        if (arm.isAtTargetState(currentShoulderTolerance, currentWristTolerance) && 
            Math.abs(Units.radiansToDegrees(arm.getWristVelocityRadiansPerSecond())) < wristVelocityTolerance.doubleValue() &&
            Math.abs(Units.radiansToDegrees(arm.getShoulderVelocityRadiansPerSecond())) < shoulderVelocityTolerance.doubleValue()) {
            // var targetState = trajectory.getStates().get(currentIndex);
            // System.out.println("COMPLETED: " + "Shoulder: " + targetState.getShoulderDegrees() + ", Wrist: " + targetState.getWristDegrees());
            currentIndex++;
        }
    }

    @Override
    public boolean isFinished() {
        return trajectory.getEntries() == 0 || currentIndex >= trajectory.getEntries();
    }

    @Override
    public void end(boolean interrupted) {
        arm.resetConstraints();
    }
    
    public ArmRoutine withTolerance(double wristTolerance, double shoulderTolerance) {
        this.wristTolerance = wristTolerance;
        this.shoulderTolerance = shoulderTolerance;
        return this;
    }

    public ArmRoutine withShoulderConstraints(double velocity, double acceleration) {
        this.shoulderMaxVelocity = velocity;
        this.shoulderMaxAcceleration = acceleration;
        return this;
    }

    public ArmRoutine withWristConstraints(double velocity, double acceleration) {
        this.wristMaxVelocity = velocity;
        this.wristMaxAcceleration = acceleration;
        return this;
    }
}
