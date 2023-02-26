package com.stuypulse.robot.commands.arm;

import static com.stuypulse.robot.constants.Settings.Operator.*;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmVoltageDrive extends CommandBase {
    
    private Arm arm;

    private IStream shoulderVoltage;
    private IStream wristVoltage;

    public ArmVoltageDrive(Gamepad gamepad) {
        arm = Arm.getInstance();

        shoulderVoltage = IStream.create(gamepad::getLeftY)
            .filtered(
                x -> MathUtil.applyDeadband(x, 0.08), 
                x -> SHOULDER_DRIVE_VOLTAGE.get()*x);

        wristVoltage = IStream.create(gamepad::getRightY)
            .filtered(
                x -> MathUtil.applyDeadband(x, 0.08), 
                x -> WRIST_DRIVE_VOLTAGE.get()*x);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setCoast(false, false);
    }

    @Override
    public void execute() {
        arm.setShoulderVoltage(shoulderVoltage.get());
        arm.setWristVoltage(wristVoltage.get());
    }

}
