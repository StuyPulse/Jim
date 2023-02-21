package com.stuypulse.robot.commands.arm;



import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.stuypulse.robot.constants.Settings.Operator.*;

public class ArmDrive extends CommandBase {
    private final Arm arm;

    private final IStream shoulder;
    private final IStream wrist;

    private final StopWatch timer;
   
    public ArmDrive(Gamepad gamepad){
        this.arm = Arm.getInstance();


        // these give values in deg / s
        this.shoulder = IStream.create(gamepad::getLeftY).filtered(
            x -> SLMath.deadband(x, DEADBAND.get()),
            x -> SLMath.spow(x, 2),
            x -> x * SHOULDER_TELEOP_SPEED.get());

        this.wrist = IStream.create(gamepad::getRightY).filtered(
            x -> SLMath.deadband(x, DEADBAND.get()),
            x -> SLMath.spow(x, 2),
            x -> x * WRIST_TELEOP_SPEED.get());

        // timer is used to get deg from deg / s (by multiplying by time)
        timer = new StopWatch();

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Manager.getInstance().setRoutine(Routine.MANUAL_CONTROL);
        timer.reset();
    }
    
    @Override
    public void execute(){
        final double dt = timer.reset();

        arm.moveShoulder(Rotation2d.fromDegrees(shoulder.get() * dt));
        arm.moveWrist(Rotation2d.fromDegrees(wrist.get() * dt));
    }

    @Override
    public boolean isFinished() {
        return Manager.getInstance().getRoutine() != Routine.MANUAL_CONTROL;
    }
}
