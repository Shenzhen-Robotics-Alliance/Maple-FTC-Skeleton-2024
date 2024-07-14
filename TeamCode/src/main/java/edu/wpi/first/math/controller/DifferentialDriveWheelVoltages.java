// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.controller;

/** Motor voltages for a differential drive. */
public class DifferentialDriveWheelVoltages {
  /** Left wheel voltage. */
  public double left;

  /** Right wheel voltage. */
  public double right;

  /** Default constructor. */
  public DifferentialDriveWheelVoltages() {}

  /**
   * Constructs a DifferentialDriveWheelVoltages.
   *
   * @param left Left wheel voltage.
   * @param right Right wheel voltage.
   */
  public DifferentialDriveWheelVoltages(double left, double right) {
    this.left = left;
    this.right = right;
  }
}
