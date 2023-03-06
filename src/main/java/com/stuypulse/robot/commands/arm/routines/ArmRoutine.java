package com.stuypulse.robot.commands.arm.routines;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class ArmRoutine extends CommandBase {

    private Number shoulderTolerance;
    private Number wristTolerance;
    
    private final Arm arm;
    protected final Supplier<ArmState> endState;
    
    protected ArmTrajectory trajectory;
    private int currentIndex;

    public ArmRoutine(Supplier<ArmState> endState) {
        arm = Arm.getInstance();

        shoulderTolerance = Shoulder.TOLERANCE;
        wristTolerance = Wrist.TOLERANCE;
        
        this.endState = endState;
        currentIndex = 0;
        

        addRequirements(arm);
    }

    protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        // TODO: check if src and dest are on the same side
        double wristSafeAngle = 90; // src.getShoulderState().getCos() > 0 ? 120 : 60;

        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), wristSafeAngle)
            .addState(dest.getShoulderDegrees(), wristSafeAngle)
            .addState(dest);
    }

    @Override
    public void initialize() {
        trajectory = getTrajectory(Arm.getInstance().getTargetState(), endState.get());
        
        // for (ArmState state : trajectory.getStates()) {
        //     System.out.println("Shoulder: " + state.getShoulderDegrees() + ", Wrist: " + state.getWristDegrees());
        // }
        // System.out.println();
        
        currentIndex = 0;
    }

    @Override
    public void execute() {
        var targetState = trajectory.getStates().get(currentIndex);
        arm.setTargetState(targetState);

        arm.setLimp(targetState.isWristLimp(), false);

        double currentShoulderTolerance = (targetState.getShoulderTolerance().orElse(shoulderTolerance)).doubleValue();
        double currentWristTolerance = (targetState.getWristTolerance().orElse(wristTolerance)).doubleValue();

        SmartDashboard.putNumber("Arm/Shoulder/Current Tolerance (deg)", currentShoulderTolerance);
        SmartDashboard.putNumber("Arm/Wrist/Current Tolerance (deg)", currentWristTolerance);

        if (targetState.isWristLimp()) {
            currentWristTolerance = 360;
        }

        if (arm.isAtTargetState(currentShoulderTolerance, currentWristTolerance)) {
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
        if (interrupted) {
            arm.setTargetState(arm.getState());
        }
    }
    
    public Command withTolerance(double wristTolerance, double shoulderTolerance) {
        this.wristTolerance = wristTolerance;
        this.shoulderTolerance = shoulderTolerance;
        return this;
    }
}
