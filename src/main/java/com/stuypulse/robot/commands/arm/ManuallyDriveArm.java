package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.IArm;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManuallyDriveArm extends CommandBase {
    private final IArm arm;

    private final IStream shoulder, wrist;

    public ManuallyDriveArm(Gamepad gamepad) {
        this.arm = IArm.getInstance();
        this.shoulder = IStream.create(() -> gamepad.getDPadX()).filtered(
            x -> x * 5
        );
        this.wrist = IStream.create(() -> gamepad.getDPadY()).filtered(
            x -> x * 5
        );
        
        addRequirements(arm);
    }

    @Override
    public void execute(){
        arm.moveShoulder(Rotation2d.fromDegrees(shoulder.get()));
        arm.moveWrist(Rotation2d.fromDegrees(wrist.get()));
    }
}
