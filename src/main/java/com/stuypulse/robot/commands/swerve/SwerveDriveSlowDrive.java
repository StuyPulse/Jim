package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveSlowDrive extends CommandBase {
    
    private final SwerveDrive swerve;
    private final Arm arm;

    private final VStream speed;
    private final IStream turn;
    private final BStream robotRelative;

    public SwerveDriveSlowDrive(Gamepad driver) {
        swerve = SwerveDrive.getInstance();
        arm = Arm.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.Drive.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.Drive.MAX_SLOW_SPEED.get())
                // new VLowPassFilter(Settings.Driver.Drive.RC),
                // new VRateLimit(Settings.Driver.Drive.MAX_TELEOP_ACCEL)
            );

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Turn.POWER.get()),
                x -> x * Settings.Driver.Turn.MAX_SLOW_TURNING.get()
                // new LowPassFilter(Settings.Driver.Turn.RC)
            );
        
        robotRelative = BStream.create(driver::getRightTriggerPressed);

        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        if (robotRelative.get()) {
            Vector2D s = speed.get();
            Vector2D translation = new Vector2D(s.y, -s.x);

            double shoulderDegNormalized = arm.getShoulderAngle().getDegrees() + 90;
            if (Math.abs(shoulderDegNormalized) > Shoulder.OVER_BUMPER_ANGLE.get() && arm.getShoulderAngle().getCos() < 0) {
                translation = translation.negative();
            }

            swerve.setChassisSpeeds(
                new ChassisSpeeds(translation.x, translation.y, -turn.get()));
        } else {
            swerve.drive(speed.get(), turn.get());
        }
    }
}