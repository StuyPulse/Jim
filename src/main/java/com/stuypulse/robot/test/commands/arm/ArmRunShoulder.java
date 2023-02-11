package com.stuypulse.robot.test.commands.arm;

import com.stuypulse.robot.TestRobotContainer;
import com.stuypulse.robot.test.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmRunShoulder extends InstantCommand{
    private Arm arm;

    public ArmRunShoulder(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute(){
        arm.runShoulder(TestRobotContainer.UNIVERSAL_VOLTAGE.get());
    }

}

