package com.stuypulse.robot.commands.wing;

import com.stuypulse.robot.subsystems.wing.Wing;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingRetract extends InstantCommand{

    public WingRetract() {
        super(() -> { Wing.getInstance().retract(); });
    }

}
