package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveSlowDrive extends CommandBase {
    
    private SwerveDrive swerve;

    private VStream speed;
    private IStream turn;

    public SwerveDriveSlowDrive(Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.SPEED_DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.MAX_SLOW_SPEED.get())
                // new VLowPassFilter(Settings.Driver.Drive.RC),
                // new VRateLimit(Settings.Driver.MAX_SLOW_ACCEL)
            );

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.ANGLE_DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Turn.POWER.get()),
                x -> x * Settings.Driver.MAX_SLOW_TURNING.get()
                // new LowPassFilter(Settings.Driver.Turn.RC)
            );
        
        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        swerve.drive(speed.get(), turn.get());
    }
}