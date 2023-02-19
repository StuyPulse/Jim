package com.stuypulse.robot.commands.arm.routines;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmBFSField;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRountine extends CommandBase {
    
    private Arm arm;
    private Manager manager;

    private Supplier<ArmBFSField> trajectory;
    private Routine routine;

    public ArmRountine(Routine routine, Supplier<ArmBFSField> trajectory) {
        arm = Arm.getInstance();
        manager = Manager.getInstance();

        this.routine = routine;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        arm.setTrajectory(trajectory.get());
        manager.setRoutine(routine);
    }

    @Override
    public boolean isFinished() {
        return arm.isArmAtTargetState(Shoulder.TOLERANCE, Wrist.TOLERANCE) || manager.getRoutine() != routine;
    }
}
