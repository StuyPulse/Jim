package com.stuypulse.robot;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.Pump;
import com.stuypulse.robot.test.commands.arm.*;
import com.stuypulse.robot.test.commands.intake.*;
import com.stuypulse.robot.test.commands.plant.*;
import com.stuypulse.robot.test.commands.swerve.*;
import com.stuypulse.robot.test.commands.wings.*;
import com.stuypulse.robot.test.subsystems.Arm;
import com.stuypulse.robot.test.subsystems.Intake;
import com.stuypulse.robot.test.subsystems.Plant;
import com.stuypulse.robot.test.subsystems.SwerveDrive;
import com.stuypulse.robot.test.subsystems.Wings;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.network.SmartNumber;

public class TestRobotContainer {

    public static SmartNumber UNIVERSAL_VOLTAGE = new SmartNumber("Universal Voltage", 0);

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

    public TestRobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // swerve
        driver.getDPadLeft().whileTrue(new TurnFrontLeft(swerve));
        driver.getDPadUp().whileTrue(new TurnFrontRight(swerve));
        driver.getDPadRight().whileTrue(new TurnBackRight(swerve));
        driver.getDPadDown().whileTrue(new TurnFrontRight(swerve));

        driver.getLeftButton().whileTrue(new DriveFrontLeft(swerve));
        driver.getTopButton().whileTrue(new DriveFrontRight(swerve));
        driver.getRightButton().whileTrue(new DriveBackRight(swerve));
        driver.getBottomButton().whileTrue(new DriveFrontRight(swerve));

        // wings
        operator.getDPadLeft().onTrue(new WingExtendLeft(wings));
        operator.getDPadUp().onTrue(new WingRetractLeft(wings));
        operator.getDPadRight().onTrue(new WingExtendRight(wings));
        operator.getDPadDown().onTrue(new WingRetractRight(wings));

        // plant
        operator.getTopButton().onTrue(new PlantDisengage(plant));
        operator.getBottomButton().onTrue(new PlantEngage(plant));
    }

    private void configureDefaultCommands() {
        // arm
        arm.setDefaultCommand(new ArmDrive(arm, driver));

        // intake
        intake.setDefaultCommand(new IntakeDrive(intake, driver));
    }
    
}
