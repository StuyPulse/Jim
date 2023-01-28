package com.stuypulse.robot.commands.Intake;

import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCube extends CommandBase{
    private IIntake intake;
    public IntakeCube(){
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.cubeIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
