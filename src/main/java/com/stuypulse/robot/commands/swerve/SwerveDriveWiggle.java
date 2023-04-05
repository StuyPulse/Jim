package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveWiggle extends CommandBase {

    private StopWatch timer;
    
    private final IStream rotation;

    private final SwerveDrive swerve;

    public SwerveDriveWiggle(double period, double radPerSecondAmplitude) {
        timer = new StopWatch();

        rotation = IStream.create(() -> {
            return Math.sin((timer.getTime() * 2 * Math.PI) / period);
        });

        this.swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, rotation.get()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
