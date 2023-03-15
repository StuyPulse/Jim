package com.stuypulse.robot.commands.swerve;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveDrive extends CommandBase {
    
    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final Plant plant;
    private final Arm arm;

    private VStream speed;
    private IStream turn;
    private BStream robotRelative;

    private final Gamepad driver;

    private final AngleController gyroFeedback;
    private Optional<Rotation2d> holdAngle;

    public SwerveDriveDrive(Gamepad driver) {
        IStream thrust = IStream.create(driver::getRightTrigger)
            .filtered(x -> MathUtil.interpolate(x, Settings.Driver.THRUST_PERCENTAGE.get() , 1.0));

        this.driver = driver;

        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();
        plant = Plant.getInstance();
        arm = Arm.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.Drive.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.Drive.MAX_TELEOP_SPEED.get()).mul(thrust.get()),
                new VRateLimit(Settings.Driver.Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Settings.Driver.Drive.RC)
            );


        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Turn.POWER.get()),
                x -> x * Settings.Driver.Turn.MAX_TELEOP_TURNING.get() * thrust.get(),
                new LowPassFilter(Settings.Driver.Turn.RC)
            );
        
        robotRelative = BStream.create(driver::getRightTriggerPressed);

        holdAngle = Optional.empty();
        gyroFeedback = new AnglePIDController(Alignment.Rotation.P, Alignment.Rotation.I, Alignment.Rotation.D);

        addRequirements(swerve, plant);
    }

    private boolean isTurnInDeadband() {
        return Math.abs(turn.get()) < Settings.Driver.Turn.DEADBAND.get();
    }
    
    @Override
    public void execute() {
        double angularVel = turn.get();

        // if turn in deadband, save the current angle and calculate small adjustments
        if (isTurnInDeadband()) {
            if (holdAngle.isEmpty()) {
                holdAngle = Optional.of(Odometry.getInstance().getRotation());
            }

            angularVel = -gyroFeedback.update(
                Angle.fromRotation2d(holdAngle.get()),
                Angle.fromRotation2d(odometry.getRotation()));
        } 
        
        // if turn outside deadband, clear the saved angle
        else {
            holdAngle = Optional.empty();
        }

        // use the angularVelocity for drive
        if (robotRelative.get()) {
            Vector2D s = speed.get();
            Vector2D translation = new Vector2D(s.y, -s.x);

            double shoulderDegNormalized = arm.getShoulderAngle().getDegrees() + 90;
            if (Math.abs(shoulderDegNormalized) > Shoulder.OVER_BUMPER_ANGLE.get() && arm.getShoulderAngle().getCos() < 0) {
                translation = translation.negative();
            }

            swerve.setChassisSpeeds(
                new ChassisSpeeds(translation.x, translation.y, angularVel));
        } else {
            swerve.drive(speed.get(), angularVel);
        }

        // unplant if driving in endgame
        if (Timer.getMatchTime() < 30) {
            if ((driver.getLeftStick().magnitude() > 0.5) ||
                (driver.getRightStick().magnitude() > 0.5)) {
             
                plant.disengage();
            }
        }
    }
}