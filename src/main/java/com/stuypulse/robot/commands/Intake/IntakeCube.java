package com.stuypulse.robot.commands.Intake;

import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeCube extends InstantCommand{
    public IIntake intake;
    public IntakeCube(IIntake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.cubeIntake();
    }
}
