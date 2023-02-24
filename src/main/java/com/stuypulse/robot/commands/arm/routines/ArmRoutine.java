package com.stuypulse.robot.commands.arm.routines;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.ArmTrajectories;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRoutine extends CommandBase {
    
    private final Arm arm;
    private final Supplier<ArmState> endState;
    
    protected ArmTrajectory trajectory;
    private int currentIndex;

    public ArmRoutine(Supplier<ArmState> endState) {
        arm = Arm.getInstance();
        
        this.endState = endState;
        currentIndex = 0;
        

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        var state = Arm.getInstance().getState();

        trajectory =
            ArmTrajectories.generateTrajectory(
                state,
                endState.get()).wristMovesUpFirst(state, Manager.getInstance().getGamePiece());

        currentIndex = 0;
    }

    @Override
    public void execute() {
        arm.setTargetState(trajectory.getStates().get(currentIndex));

        if (arm.isAtTargetState(Shoulder.TOLERANCE.get(), Wrist.TOLERANCE.get())) {
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
