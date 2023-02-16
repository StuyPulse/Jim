package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDrivePlantDrive extends CommandBase {
    
    private SwerveDrive swerve;
    private Plant plant;

    private VStream speed;
    private IStream turn;

    private BStream planting;

    public SwerveDrivePlantDrive(Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();
        this.plant = Plant.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.SPEED_DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.MAX_TELEOP_SPEED.get()),
                new VLowPassFilter(Settings.Driver.Drive.RC),
                new VRateLimit(Settings.Driver.MAX_TELEOP_ACCEL)
            );

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.ANGLE_DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Turn.POWER.get()),
                x -> x * Settings.Driver.MAX_TELEOP_TURNING.get(),
                new LowPassFilter(Settings.Driver.Turn.RC)
            );

        planting = BStream.create(() -> speed.get().magnitude() < 0.05 && turn.get() < 0.05)
            .filtered(new BDebounce.Rising(Settings.Driver.PLANT_DEBOUNCE));

        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        if (planting.get()) {
            plant.engage();
        } else {
            plant.disengage();
        }

        swerve.drive(speed.get(), turn.get());
    }
}