package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.IIntake;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCube extends CommandBase{
    private Intake intake;
    public IntakeCube(){
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        System.out.println("INTAKING CUBE");
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
