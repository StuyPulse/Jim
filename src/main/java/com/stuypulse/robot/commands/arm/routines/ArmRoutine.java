package com.stuypulse.robot.commands.arm.routines;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class ArmRoutine extends CommandBase {
    
    private final Arm arm;
    protected final Supplier<ArmState> endState;
    
    protected ArmTrajectory trajectory;
    private int currentIndex;

    public ArmRoutine(Supplier<ArmState> endState) {
        arm = Arm.getInstance();
        
        this.endState = endState;
        currentIndex = 0;
        

        addRequirements(arm);
    }

    protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        return new ArmTrajectory()
            .addState(src.getShoulderDegrees(), 90)
            .addState(dest.getShoulderDegrees(), 90)
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
        arm.setTargetState(trajectory.getStates().get(currentIndex));

        if (arm.isAtTargetState(Shoulder.TOLERANCE.get(), Wrist.TOLERANCE.get())) {
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
}
