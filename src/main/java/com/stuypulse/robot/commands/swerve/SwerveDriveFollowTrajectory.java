/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

public class SwerveDriveFollowTrajectory extends FollowPathHolonomic {

	public static HashMap<String, PathPlannerPath> getSeparatedPaths(List<PathPlannerPath> paths, String... names) {
		if (paths.size() != names.length)
			throw new IllegalArgumentException("Invalid number of path names given to FollowTrajectory.getSeparatedPaths");

		HashMap<String, PathPlannerPath> map = new HashMap<String, PathPlannerPath>();

		for (int i = 0; i < names.length; i++) {
			map.put(names[i], paths.get(i));
		}

		return map;
	}

	private final PathPlannerPath path;

	private boolean robotRelative;
	private boolean shouldStop;

	private FieldObject2d trajectory;

	public SwerveDriveFollowTrajectory(PathPlannerPath path) {
		super(
			path,
			Odometry.getInstance()::getPose,
			SwerveDrive.getInstance()::getChassisSpeeds,
			SwerveDrive.getInstance()::setChassisSpeeds,
			Motion.XY,
			Motion.THETA,
			Settings.Swerve.MAX_MODULE_SPEED.get(),
			Settings.Swerve.RADIUS,
			new ReplanningConfig(),
			() -> {
				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
			SwerveDrive.getInstance()
		);

		this.path = path;

		robotRelative = false;
		trajectory = Odometry.getInstance().getField().getObject("Trajectory");
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

	@Override
	public void initialize() {
		PathPlannerTrajectory generatedTrajectory = new PathPlannerTrajectory(
			path, SwerveDrive.getInstance().getChassisSpeeds(), Odometry.getInstance().getRotation());

		if (robotRelative) {
			PathPlannerTrajectory.State initialState = generatedTrajectory.getInitialState();
			Pose2d initialPose = new Pose2d(
				initialState.positionMeters,
				initialState.heading);

			Optional<Alliance> alliance = DriverStation.getAlliance();

			if (alliance.isPresent() && alliance.get() == Alliance.Red) {
				initialPose = GeometryUtil.flipFieldPose(initialPose);
			}

			Odometry.getInstance().reset(initialPose);
		}

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
