// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinematics;

import static edu.wpi.first.units.Units.Meters;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/** Represents the wheel positions for a differential drive drivetrain. */
public class DifferentialDriveWheelPositions
    implements WheelPositions<DifferentialDriveWheelPositions> {
  /** Distance measured by the left side. */
  public double leftMeters;

  /** Distance measured by the right side. */
  public double rightMeters;

  /**
   * Constructs a DifferentialDriveWheelPositions.
   *
   * @param leftMeters Distance measured by the left side.
   * @param rightMeters Distance measured by the right side.
   */
  public DifferentialDriveWheelPositions(double leftMeters, double rightMeters) {
    this.leftMeters = leftMeters;
    this.rightMeters = rightMeters;
  }

  /**
   * Constructs a DifferentialDriveWheelPositions.
   *
   * @param left Distance measured by the left side.
   * @param right Distance measured by the right side.
   */
  public DifferentialDriveWheelPositions(Measure<Distance> left, Measure<Distance> right) {
    this(left.in(Meters), right.in(Meters));
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof DifferentialDriveWheelPositions
        && Math.abs(((DifferentialDriveWheelPositions) obj).leftMeters - leftMeters) < 1E-9
        && Math.abs(((DifferentialDriveWheelPositions) obj).rightMeters - rightMeters) < 1E-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(leftMeters, rightMeters);
  }

  @NonNull
  @SuppressLint("DefaultLocale")
  @Override
  public String toString() {
    return String.format(
        "DifferentialDriveWheelPositions(Left: %.2f m, Right: %.2f m", leftMeters, rightMeters);
  }

  @Override
  public DifferentialDriveWheelPositions copy() {
    return new DifferentialDriveWheelPositions(leftMeters, rightMeters);
  }

  @Override
  public DifferentialDriveWheelPositions interpolate(
      DifferentialDriveWheelPositions endValue, double t) {
    return new DifferentialDriveWheelPositions(
        MathUtil.interpolate(this.leftMeters, endValue.leftMeters, t),
        MathUtil.interpolate(this.rightMeters, endValue.rightMeters, t));
  }
}
