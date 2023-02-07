package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.stuypulse.robot.constants.Settings;

public class ArmDrive extends CommandBase {
    private final Arm arm;
    private final IStream shoulder, wrist;
   
    public ArmDrive(Gamepad gamepad){
        this.arm = Arm.getInstance();

        this.shoulder = IStream.create(() -> gamepad.getLeftY()).filtered(
            x -> SLMath.spow(x, 2) * Settings.Arm.Shoulder.shoulderSpeedDegrees.get()
        );
        this.wrist = IStream.create(() -> gamepad.getRightY()).filtered(
            x -> SLMath.spow(x, 2) * Settings.Arm.Wrist.wrist   SpeedDegrees.get()
        );

        addRequirements(arm);
    }
    
    @Override
    public void execute(){
        arm.moveShoulder(Rotation2d.fromDegrees(shoulder.get()));
        arm.moveWrist(Rotation2d.fromDegrees(wrist.get()));
    }
}
