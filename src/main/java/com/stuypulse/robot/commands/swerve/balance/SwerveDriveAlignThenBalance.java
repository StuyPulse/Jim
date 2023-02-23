package com.stuypulse.robot.commands.swerve.balance;

import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.swerve.SwerveDrivePointWheels;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveAlignThenBalance extends SequentialCommandGroup {

    public SwerveDriveAlignThenBalance() {


        addCommands(
            new LEDSet(LEDColor.RED.pulse()),
            new SwerveDriveBalanceAlign(),
            new SwerveDriveBalanceBlay(),
            new SwerveDrivePointWheels(Rotation2d.fromDegrees(90)),
            new LEDSet(LEDColor.BLUE)
        );
    }
}