/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetNodeLevel extends InstantCommand {

    public ManagerSetNodeLevel(NodeLevel nodeLevel) {
        super(() -> Manager.getInstance().setNodeLevel(nodeLevel));
    }

}
