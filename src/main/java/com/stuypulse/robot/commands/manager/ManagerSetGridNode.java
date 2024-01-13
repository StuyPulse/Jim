/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.manager;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.subsystems.Manager;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetGridNode extends InstantCommand {

    public ManagerSetGridNode(int index) {
        super(() -> {
            var manager = Manager.getInstance();

            if (RobotContainer.getCachedAlliance().get() == Alliance.Blue) {
                manager.setGridNode(index);
            } else {
                manager.setGridNode(8 - index);
            }
        });
    }
}
