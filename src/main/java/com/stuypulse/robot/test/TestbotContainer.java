package com.stuypulse.robot.test;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.Pump;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;

public class TestbotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);
    
    // // Subsystem
    public final Intake intake = new Intake();
    public final SwerveDrive swerve = new SwerveDrive();
    public final Arm arm = new Arm();
    public final Plant plant = new Plant();
    public final Wings wings = new Wings();
    
    public final Pump pump = new Pump();

    public TestbotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // swerve
        driver.getDPadLeft()
            .onTrue(swerve.runOnce(swerve::turnMotorFL))
            .onFalse(swerve.runOnce(swerve::stop));

        driver.getDPadRight()
            .onTrue(swerve.runOnce(swerve::turnMotorFR))
            .onFalse(swerve.runOnce(swerve::stop));

        driver.getDPadDown()
            .onTrue(swerve.runOnce(swerve::turnMotorBR))
            .onFalse(swerve.runOnce(swerve::stop));
        
        driver.getDPadUp()
            .onTrue(swerve.runOnce(swerve::turnMotorBL))
            .onFalse(swerve.runOnce(swerve::stop));

        driver.getLeftButton()
            .onTrue(swerve.runOnce(swerve::driveMotorFL))
            .onFalse(swerve.runOnce(swerve::stop));

        driver.getTopButton()
            .onTrue(swerve.runOnce(swerve::driveMotorFR))
            .onFalse(swerve.runOnce(swerve::stop));

        driver.getRightButton()
            .onTrue(swerve.runOnce(swerve::driveMotorBR))
            .onFalse(swerve.runOnce(swerve::stop));

        driver.getBottomButton()
            .onTrue(swerve.runOnce(swerve::driveMotorFR))
            .onFalse(swerve.runOnce(swerve::stop));

        // wings
        operator.getDPadLeft().onTrue(wings.runOnce(wings::extendLeftDeploy));
        operator.getDPadUp().onTrue(wings.runOnce(wings::retractLeftDeploy));
        operator.getDPadRight().onTrue(wings.runOnce(wings::extendRightDeploy));
        operator.getDPadDown().onTrue(wings.runOnce(wings::retractRightDeploy));

        operator.getLeftButton().onTrue(wings.runOnce(wings::extendLeftLatch));
        operator.getTopButton().onTrue(wings.runOnce(wings::retractLeftLatch));
        operator.getRightButton().onTrue(wings.runOnce(wings::extendRightLatch));
        operator.getBottomButton().onTrue(wings.runOnce(wings::retractRightLatch));


        // plant
        operator.getLeftBumper().onTrue(plant.runOnce(plant::disengage));
        operator.getRightBumper().onTrue(plant.runOnce(plant::engage));

        // intake
        driver.getLeftTriggerButton()
            .onTrue(intake.runOnce(intake::runFront))
            .onFalse(intake.runOnce(intake::stop));

        driver.getRightTriggerButton()
            .onTrue(intake.runOnce(intake::runBack))
            .onFalse(intake.runOnce(intake::stop));
    }

    private void configureDefaultCommands() {
        // arm
        arm.setDefaultCommand(arm.run(() -> {
            double shoulderVolts = MathUtil.applyDeadband(operator.getLeftY(), 0.05) * 10;
            double wristVolts = MathUtil.applyDeadband(operator.getRightY(), 0.05) * 10;

            arm.runShoulder(shoulderVolts);
            arm.runWrist(wristVolts);
        }));
    }
    
}
