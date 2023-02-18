package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmSetRoutine extends CommandBase {
    
    private Arm arm;
    private Manager manager;
    private Routine routine;

    public ArmSetRoutine(Routine routine) {
        arm = Arm.getInstance();
        manager = Manager.getInstance();

        this.routine = routine;
    }

    @Override
    public void initialize() {
        manager.setRoutine(routine);
    }

    @Override
    public boolean isFinished() {
        // TODO: this is not the real final setpoint (this is along the arm bfs field)
        return manager.getRoutine() != routine || arm.isArmAtTargetState(); 

    }
}
