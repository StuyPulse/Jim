/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerFlipScoreSide extends InstantCommand {
    public ManagerFlipScoreSide() {
        super(() -> {
            var manager = Manager.getInstance();
            if (manager.getScoreSide() == ScoreSide.FRONT) {
                manager.setScoreSide(ScoreSide.BACK);
            } else {
                manager.setScoreSide(ScoreSide.BACK);
            }
        });
    }
}
