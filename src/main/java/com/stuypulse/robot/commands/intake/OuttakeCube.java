package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.IIntake;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeCube extends CommandBase{
    private IIntake intake;
    public OuttakeCube() {
        intake = IIntake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
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
