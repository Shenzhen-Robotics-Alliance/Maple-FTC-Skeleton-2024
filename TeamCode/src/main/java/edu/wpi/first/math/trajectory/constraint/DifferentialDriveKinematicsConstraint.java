// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory.constraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * A class that enforces constraints on the differential drive kinematics. This can be used to
 * ensure that the trajectory is constructed so that the commanded velocities for both sides of the
 * drivetrain stay below a certain limit.
 */
public class DifferentialDriveKinematicsConstraint implements TrajectoryConstraint {
  private final double m_maxSpeedMetersPerSecond;
  private final DifferentialDriveKinematics m_kinematics;

  /**
   * Constructs a differential drive dynamics constraint.
   *
   * @param kinematics A kinematics component describing the drive geometry.
   * @param maxSpeedMetersPerSecond The max speed that a side of the robot can travel at.
   */
  public DifferentialDriveKinematicsConstraint(
      final DifferentialDriveKinematics kinematics, double maxSpeedMetersPerSecond) {
    m_maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    m_kinematics = kinematics;
  }

  /**
   * Returns the max velocity given the current pose and curvature.
   *
   * @param poseMeters The pose at the current point in the trajectory.
   * @param curvatureRadPerMeter The curvature at the current point in the trajectory.
   * @param velocityMetersPerSecond The velocity at the current point in the trajectory before
   *     constraints are applied.
   * @return The absolute maximum velocity.
   */
  @Override
  public double getMaxVelocityMetersPerSecond(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    // Create an object to represent the current chassis speeds.
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(
            velocityMetersPerSecond, 0, velocityMetersPerSecond * curvatureRadPerMeter);

    // Get the wheel speeds and normalize them to within the max velocity.
    edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(m_maxSpeedMetersPerSecond);

    // Return the new linear chassis speed.
    return m_kinematics.toChassisSpeeds(wheelSpeeds).vxMetersPerSecond;
  }

  /**
   * Returns the minimum and maximum allowable acceleration for the trajectory given pose,
   * curvature, and speed.
   *
   * @param poseMeters The pose at the current point in the trajectory.
   * @param curvatureRadPerMeter The curvature at the current point in the trajectory.
   * @param velocityMetersPerSecond The speed at the current point in the trajectory.
   * @return The min and max acceleration bounds.
   */
  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    return new MinMax();
  }
}
