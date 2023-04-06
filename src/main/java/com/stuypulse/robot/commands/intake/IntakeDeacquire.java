/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.intake.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeDeacquire extends InstantCommand {
    private Intake intake;

    public IntakeDeacquire(){
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.deacquire();
    }
}
