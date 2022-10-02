// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.rambots4571.rampage.vision.Limelight.CamMode;
import com.rambots4571.rampage.vision.Limelight.LedMode;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Shooters;

public class Vision extends SubsystemBase {
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
  

  /** Creates a new Limelight. */
  public Vision() {
  }

  // Gets the number of valid targets (0 or 1)
  public static boolean hasValidTarget() {
    return table.getEntry("tv").getNumber(0.0).intValue() == 1;
  }

  // Gets the horizontal offset of the center crosshair from the target
  public static double getHorizontalOffset() {
    return table.getEntry("tx").getDouble(Integer.MAX_VALUE);
  }

  // Gets the vertical offset of the center crosshair from the target
  public static double getVerticalOffset() {
    return table.getEntry("ty").getDouble(Integer.MAX_VALUE);
  }

  // Gets the estimated distance from the robot to the target
  public double getEstimatedDistance() {
    double offset = getVerticalOffset();
    // return 0.0;
    return Shooters.LIMELIGHT_DISTANCE_K[0] * Math.pow(offset, 2)
      + Shooters.LIMELIGHT_DISTANCE_K[1] * offset + Shooters.LIMELIGHT_DISTANCE_K[2];
  }

  // Gets the active pipeline (0 to 9)
  public int getPipe() {
    return table.getEntry("rambotspipeline").getNumber(-1).intValue();
  }

  // Sets the active pipeline (0 to 9)
  public void setPipe(int pipeNum) {
    table.getEntry("pipeline").setNumber(pipeNum);
    
  }

  // Sets the LED state. LEDs on = 3, LEDs off = 1
  public void setLEDMode(LedMode mode) {
    table.getEntry("ledmode").setNumber(3);
  }

  // Set's the limelight's state of operation. Visiom Processing = 0, Driver
  // Camera = 1
  public void setCameraMode(CamMode mode) {
    table.getEntry("camMode").setNumber(0);
  }

  
  public static void upadateTelemetry() {
    SmartDashboard.putBoolean("Limelight Valid Target", hasValidTarget());
    SmartDashboard.putNumber("xOff", getHorizontalOffset());
    SmartDashboard.putNumber("yOff", getVerticalOffset());
  }
}
