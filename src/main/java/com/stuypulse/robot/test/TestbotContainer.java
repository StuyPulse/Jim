package com.stuypulse.robot.test;

import com.stuypulse.robot.commands.odometry.OdometryRealign;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Pump;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestbotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);
    
    // // Subsystem
    public final Intake intake = Intake.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final TestArm arm = new TestArm();
    public final TestPlant plant = new TestPlant();
    public final TestWings wings = new TestWings();
    
    public final Pump pump = new Pump();

    public TestbotContainer() {
        configureDefaultCommands();
        configureButtonBindings();

        DriverStation.silenceJoystickConnectionWarning(true);

        CameraServer.startAutomaticCapture();
    }

    private void configureButtonBindings() {
        driver.getDPadDown().onTrue(new OdometryRealign());
        // swerve
        // driver.getDPadLeft()
        //     .onTrue(swerve.runOnce(swerve::turnMotorFL))
        //     .onFalse(swerve.runOnce(swerve::stop));

        // driver.getDPadRight()
        //     .onTrue(swerve.runOnce(swerve::turnMotorFR))
        //     .onFalse(swerve.runOnce(swerve::stop));

        // driver.getDPadDown()
        //     .onTrue(swerve.runOnce(swerve::turnMotorBR))
        //     .onFalse(swerve.runOnce(swerve::stop));
        
        // driver.getDPadUp()
        //     .onTrue(swerve.runOnce(swerve::turnMotorBL))
        //     .onFalse(swerve.runOnce(swerve::stop));

        // driver.getLeftButton()
        //     .onTrue(swerve.runOnce(swerve::driveMotorFL))
        //     .onFalse(swerve.runOnce(swerve::stop));

        // driver.getTopButton()
        //     .onTrue(swerve.runOnce(swerve::driveMotorFR))
        //     .onFalse(swerve.runOnce(swerve::stop));

        // driver.getRightButton()
        //     .onTrue(swerve.runOnce(swerve::driveMotorBR))
        //     .onFalse(swerve.runOnce(swerve::stop));

        // driver.getBottomButton()
        //     .onTrue(swerve.runOnce(swerve::driveMotorBL))
        //     .onFalse(swerve.runOnce(swerve::stop));

        // wings
        // operator.getDPadLeft().onTrue(wings.runOnce(wings::extendLeftDeploy));
        // operator.getDPadUp().onTrue(wings.runOnce(wings::retractLeftDeploy));
        // operator.getDPadRight().onTrue(wings.runOnce(wings::extendRightDeploy));
        // operator.getDPadDown().onTrue(wings.runOnce(wings::retractRightDeploy));

        operator.getLeftButton().onTrue(wings.runOnce(wings::extendLeftLatch));
        operator.getTopButton().onTrue(wings.runOnce(wings::retractLeftLatch));
        operator.getRightButton().onTrue(wings.runOnce(wings::extendRightLatch));
        operator.getBottomButton().onTrue(wings.runOnce(wings::retractRightLatch));


        // plant
        operator.getLeftBumper().onTrue(plant.runOnce(plant::disengage));
        operator.getRightBumper().onTrue(plant.runOnce(plant::engage));

        // intake
        // driver.getLeftTriggerButton()
        //     .onTrue(intake.runOnce(intake::runFront))
        //     .onFalse(intake.runOnce(intake::stop));

        // driver.getRightTriggerButton()
        //     .onTrue(intake.runOnce(intake::runBack))
        //     .onFalse(intake.runOnce(intake::stop));

        operator.getDPadUp()
            .onTrue(intake.runOnce(intake::acquireCone))
            .onFalse(intake.runOnce(intake::stop));
        operator.getDPadDown()
            .onTrue(intake.runOnce(intake::deacquireCone))
            .onFalse(intake.runOnce(intake::stop));
        operator.getDPadLeft()
            .onTrue(intake.runOnce(intake::acquireCube))
            .onFalse(intake.runOnce(intake::stop));
        operator.getDPadRight()
            .onTrue(intake.runOnce(intake::deacquireCube))
            .onFalse(intake.runOnce(intake::stop));
    }

    private final SmartNumber SHOULDER_VOLTS = new SmartNumber("Arm/Shoulder Input Volts", 0);
    private final SmartNumber WRIST_VOLTS = new SmartNumber("Arm/Wrist Input Volts", 0);
    private final SmartBoolean ARM_DRIVE = new SmartBoolean("Arm/Arm Drive", false);

    private final IStream shoulder  = IStream.create(operator::getLeftY).filtered(
        x -> SLMath.deadband(x, Settings.Operator.DEADBAND.get()),
        x -> SLMath.spow(x, 2),
        x -> x * Settings.Operator.SHOULDER_TELEOP_SPEED.get());

    private final IStream wrist = IStream.create(operator::getRightY).filtered(
        x -> SLMath.deadband(x, Settings.Operator.DEADBAND.get()),
        x -> SLMath.spow(x, 2),
        x -> x * Settings.Operator.WRIST_TELEOP_SPEED.get());

    private void configureDefaultCommands() {

        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        // arm
        arm.setDefaultCommand(arm.run(() -> {
            if (!SmartDashboard.getBoolean("Arm/Setpoint Control", false)) {
                if (ARM_DRIVE.get()) {
                    double shoulderVolts = MathUtil.applyDeadband(operator.getLeftY(), 0.05) * 3;
                    double wristVolts = MathUtil.applyDeadband(operator.getRightY(), 0.05) * 12;

                    
                    if (MathUtil.inputModulus(arm.getShoulderAngle().getDegrees(), -180, 180)  > 0) {
                        shoulderVolts = 0;
                    }
        
                    Settings.putNumber("Arm/Shoulder Voltage", shoulderVolts);
                    Settings.putNumber("Arm/Wrist Voltage", wristVolts);
        
                    arm.runShoulder(shoulderVolts);
                    arm.runWrist(wristVolts);
                } else {
                    arm.runShoulder(MathUtil.clamp(SHOULDER_VOLTS.get(), -3, 3));
                    arm.runWrist(MathUtil.clamp(WRIST_VOLTS.get(), -5, 5));
                }
            } else {
                if (ARM_DRIVE.get()) {
                    arm.moveShoulder(Rotation2d.fromDegrees(shoulder.get() * Settings.DT));
                    arm.moveWrist(Rotation2d.fromDegrees(wrist.get() * Settings.DT));
                }
            }
        }));
    }
    
}
