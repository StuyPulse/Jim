package com.stuypulse.robot.test.commands.intake;

import com.stuypulse.robot.TestRobotContainer;
import com.stuypulse.robot.test.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSpinFront extends InstantCommand{
    private Intake intake;

    public IntakeSpinFront(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.setFrontMotor(TestRobotContainer.UNIVERSAL_VOLTAGE.get());
    }

}

