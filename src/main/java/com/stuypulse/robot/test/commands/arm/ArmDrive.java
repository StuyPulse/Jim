package com.stuypulse.robot.test.commands.arm;

import com.stuypulse.robot.test.subsystems.Arm;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmDrive extends CommandBase {

    private Arm arm;
    private IStream shoulderVolts;
    private IStream wristVolts;

    public ArmDrive(Arm arm, Gamepad gamepad) {
        this.arm = arm;

        shoulderVolts = IStream.create(gamepad::getRightY)
        .filtered(
                x -> SLMath.deadband(x, 0.05),
                x -> x * 12);
        
        wristVolts = IStream.create(gamepad::getLeftY)
            .filtered(
                x -> SLMath.deadband(x, 0.05),
                x -> x * 12);

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.runShoulder(shoulderVolts.get());
        arm.runWrist(wristVolts.get());
    }
    
}
