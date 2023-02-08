package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetIntakeSide extends InstantCommand {
    private final Manager manager;
    private final IntakeSide intakeSide; 

    public ManagerSetIntakeSide(IntakeSide intakeSide) {
        manager = Manager.getInstance();        
        this.intakeSide = intakeSide;
        // addRequirements(manager);
    }

    @Override
    public void initialize() {
        manager.setIntakeSide(intakeSide);
    }
}