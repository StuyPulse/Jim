package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeDeacquire extends InstantCommand {
    private Intake intake;
    private Manager manager;

    public IntakeDeacquire(){
        intake = Intake.getInstance();
        manager = Manager.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        if (manager.getGamePiece().isCube()) {
            intake.deacquireCube();
        } else {
            intake.deacquireCone();
        }
    }
}
