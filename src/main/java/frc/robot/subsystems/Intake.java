// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  WPI_TalonFX intake;
  
  public Intake() {
    intake = new WPI_TalonFX(Constants.INTAKE1);
  }

  @Override
  public void periodic() {
    
  }
  public void intakeBall(XboxController controller, double speed)
  {
    intake.set(controller.getRawAxis(Constants.RIGHT_TRIGGER)*speed);
  }
  public void stop() {
    
    intake.set(0);
  }
}
