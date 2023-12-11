/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import java.util.HashMap;
import java.util.List;

public class SwerveDriveFollowTrajectory extends PPSwerveControllerCommand {

	public static HashMap<String, PathPlannerTrajectory> getSeparatedPaths(List<PathPlannerTrajectory> paths, String... names) {
		if (paths.size() != names.length)
			throw new IllegalArgumentException("Invalid number of path names given to FollowTrajectory.getSeparatedPaths");

		HashMap<String, PathPlannerTrajectory> map = new HashMap<String, PathPlannerTrajectory>();

		for (int i = 0; i < names.length; i++) {
			map.put(names[i], paths.get(i));
		}

		return map;
	}

	private boolean robotRelative;
	private boolean shouldStop;
	private PathPlannerTrajectory path;
	private HashMap<String, Command> events;

	private FieldObject2d trajectory;

	public SwerveDriveFollowTrajectory(PathPlannerTrajectory path) {

		super(
			path,
			AbstractOdometry.getInstance()::getPose,
			SwerveDrive.getInstance().getKinematics(),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.THETA.kP, Motion.THETA.kI, Motion.THETA.kD),
			SwerveDrive.getInstance()::setModuleStates,
			true,
			SwerveDrive.getInstance()
		);

		robotRelative = false;
		trajectory = AbstractOdometry.getInstance().getField().getObject("Trajectory");
		this.path = path;
		events = new HashMap<String, Command>();
		shouldStop = false;
	}

	public SwerveDriveFollowTrajectory withStop() {
		shouldStop = true;
		return this;
	}

	public SwerveDriveFollowTrajectory robotRelative() {
		robotRelative = true;
		return this;
	}

	public SwerveDriveFollowTrajectory fieldRelative() {
		robotRelative = false;
		return this;
	}

	public SwerveDriveFollowTrajectory addEvent(String name, Command command) {
		events.put(name, command);
		return this;
	}

	// FINISHES AT END OF PATH FOLLOWING, NOT AFTER ALL EVENTS DONE
	public FollowPathWithEvents withEvents() {
		return new FollowPathWithEvents(
			this,
			path.getMarkers(),
			events
		);
	}

	@Override
	public void initialize() {
		if (robotRelative) {
			PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(
				path.getInitialState(), DriverStation.getAlliance());

			AbstractOdometry.getInstance().reset(new Pose2d(
				initialState.poseMeters.getTranslation(),
				initialState.holonomicRotation
			));
		}

		trajectory.setTrajectory(PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance()));

		super.initialize();
	}

	@Override
	public void end(boolean interrupted) {
		if (shouldStop) {
			SwerveDrive.getInstance().setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
		}
		trajectory.setPose(
			new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN))
		);
	}

}
