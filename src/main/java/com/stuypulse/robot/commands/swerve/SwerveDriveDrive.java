package com.stuypulse.robot.commands.swerve;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Driver.Turn.GyroFeedback;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.arm.Arm;
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
            .filtered(x -> MathUtil.interpolate(Settings.Driver.THRUST_PERCENTAGE.get(), 1.0 , x));

        this.driver = driver;

        swerve = SwerveDrive.getInstance();
        plant = Plant.getInstance();
        arm = Arm.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()).mul(thrust.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC)
            );


        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * Turn.MAX_TELEOP_TURNING.get() * thrust.get(),
                new LowPassFilter(Turn.RC)
            );
        
        robotRelative = BStream.create(() -> false /*driver::getRightTriggerPressed*/);

        holdAngle = Optional.empty();
        gyroFeedback = new AnglePIDController(GyroFeedback.P, GyroFeedback.I, GyroFeedback.D);

        addRequirements(swerve, plant);
    }

    private boolean isTurnInDeadband() {
        return Math.abs(turn.get()) < Turn.DEADBAND.get();
    }

    private boolean isDriveInDeadband() {
        return driver.getLeftStick().magnitude() < Drive.DEADBAND.get();
    }
    
    @Override
    public void execute() {
        double angularVel = turn.get();

        // if turn in deadband, save the current angle and calculate small adjustments
        if (isTurnInDeadband()) {
            if (holdAngle.isEmpty()) {
                holdAngle = Optional.of(swerve.getGyroAngle());
            }

            if (GyroFeedback.GYRO_FEEDBACK_ENABLED.get() && !isDriveInDeadband()) {
                angularVel = -gyroFeedback.update(
                    Angle.fromRotation2d(holdAngle.get()),
                    Angle.fromRotation2d(swerve.getGyroAngle()));
            }
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