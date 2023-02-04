package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.IIntake;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeCone extends CommandBase{
    private Intake intake;
    public OuttakeCone(){
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.coneOuttake();
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
