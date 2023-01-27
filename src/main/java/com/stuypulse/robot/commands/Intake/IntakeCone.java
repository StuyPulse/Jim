package com.stuypulse.robot.commands.Intake;
import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeCone extends InstantCommand {
    private IIntake intake;

    public IntakeCone(IIntake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.coneIntake();
    }
}
