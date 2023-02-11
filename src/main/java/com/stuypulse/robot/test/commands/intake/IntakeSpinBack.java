package com.stuypulse.robot.test.commands.intake;

import com.stuypulse.robot.test.subsystems.Intake;
import com.stuypulse.robot.TestRobotContainer;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSpinBack extends InstantCommand{
    private Intake intake;

    public IntakeSpinBack(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.setBackMotor(TestRobotContainer.UNIVERSAL_VOLTAGE.get());
    }

}

