package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveDrive extends CommandBase {
    
    private final SwerveDrive swerve;
    private final Plant plant;
    private final Arm arm;

    private VStream speed;
    private IStream turn;
    private BStream robotRelative;

    private final Gamepad driver;

    public SwerveDriveDrive(Gamepad driver) {
        this.driver = driver;
        swerve = SwerveDrive.getInstance();
        arm = Arm.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.Drive.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.Drive.MAX_TELEOP_SPEED.get()),
                new VLowPassFilter(Settings.Driver.Drive.RC),
                new VRateLimit(Settings.Driver.Drive.MAX_TELEOP_ACCEL)
            );

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Turn.POWER.get()),
                x -> x * Settings.Driver.Turn.MAX_TELEOP_TURNING.get(),
                new LowPassFilter(Settings.Driver.Turn.RC)
            );
        
        robotRelative = BStream.create(() -> false /*driver::getRightTriggerPressed*/);

        plant = Plant.getInstance();
        addRequirements(swerve, plant);
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

        if (Timer.getMatchTime() < 30) {
            if ((driver.getLeftStick().magnitude() > 0.5) ||
                (driver.getRightStick().magnitude() > 0.5)) {
             
                plant.disengage();
            }
        }
    }
}