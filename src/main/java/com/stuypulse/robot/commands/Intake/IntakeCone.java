package com.stuypulse.robot.commands.Intake;
import com.stuypulse.robot.subsystems.intake.IIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCone extends CommandBase{
    private IIntake intake;

    public IntakeCone(IIntake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.coneIntake();
    }
    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }
}
