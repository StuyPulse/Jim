package com.stuypulse.robot.commands.manager;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManagerSetIntakeSide extends InstantCommand {

    public ManagerSetIntakeSide(IntakeSide intakeSide) {
        super(() -> Manager.getInstance().setIntakeSide(intakeSide));
    }

}