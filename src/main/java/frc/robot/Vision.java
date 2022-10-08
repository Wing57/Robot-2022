// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.vision.Limelight;
import com.rambots4571.rampage.vision.Limelight.CamMode;
import com.rambots4571.rampage.vision.Limelight.LedMode;
import com.rambots4571.rampage.vision.ReadValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.Shooters;

public class Vision implements Sendable {
  private static final Limelight limelight = Limelight.getInstance();
  private static Vision instance;

  private Vision() {
  }

  public static Vision getInstance() {
    if (instance == null)
      instance = new Vision();
    return instance;
  }

  // Gets the number of valid targets (0 or 1)
  public boolean hasValidTarget() {
    return limelight.getValue(ReadValue.tv) == 1;
  }

  // Gets the horizontal offset of the center crosshair from the target
  public double getHorizontalOffset() {
    return limelight.getValue(ReadValue.tx);
  }

  // Gets the vertical offset of the center crosshair from the target
  public double getVerticalOffset() {
    return limelight.getValue(ReadValue.ty);
  }

  // Gets the estimated distance from the robot to the target
  public double getEstimatedDistance() {
    double offset = getVerticalOffset();
    double angleGoalDegrees = Shooters.MountAngleDegrees - offset;
    double angleGoalRadians = angleGoalDegrees * (Math.PI / 180.0);
    double estimatedDistance = (Shooters.GoalHeightInches - Shooters.LensHeightInches)
      / Math.tan(angleGoalRadians);
    return estimatedDistance;
  }

  // Gets the active pipeline (0 to 9)
  public int getPipe() {
    return limelight.getEntry("rambotspipeline").getNumber(-1).intValue();
  }

  // Sets the active pipeline (0 to 9)
  public void setPipe(int pipeNum) {
    limelight.setPipeline(pipeNum);
  }

  // Sets the LED state. LEDs on = 3, LEDs off = 1
  public void setLEDMode(LedMode mode) {
    limelight.setLedMode(mode);
  }

  // Set's the limelight's state of operation. Visiom Processing = 0, Driver
  // Camera = 1
  public void setCameraMode(CamMode mode) {
    limelight.setCamMode(mode);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Limelight Values");
    builder.addBooleanProperty("has valid target", this::hasValidTarget, null);
    builder.addDoubleProperty("xOff", this::getHorizontalOffset, null);
    builder.addDoubleProperty("yOff", this::getVerticalOffset, null);
    builder.addDoubleProperty("estimate distance", this::getEstimatedDistance, null);
  }
}
