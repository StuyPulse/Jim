/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*-
 * This auton does nothing... it is used as a placeholder
 */
public class DoNothing extends SequentialCommandGroup {

    public DoNothing() {
        addCommands(
                /** Do a whole lot of nothing */
                );
    }
}
