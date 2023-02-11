package com.stuypulse.robot.test.commands.intake;

import com.stuypulse.robot.test.subsystems.Intake;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeDrive extends CommandBase {
    
    private Intake intake;
    private IStream frontVoltage;
    private IStream backVoltage;
    
    public IntakeDrive(Intake intake, Gamepad gamepad) {
        this.intake = intake;

        frontVoltage = IStream.create(gamepad::getRightTrigger)
            .filtered(x -> x * 12);
        
        backVoltage = IStream.create(gamepad::getLeftTrigger)
            .filtered(x -> x * 12);
    }

    @Override
    public void execute() {
        intake.setFrontMotor(frontVoltage.get());
        intake.setBackMotor(backVoltage.get());
    }

}
