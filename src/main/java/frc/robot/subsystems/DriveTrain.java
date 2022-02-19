// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    // Local Talon Variables
    private final WPI_TalonFX rightMaster;
    private final WPI_TalonFX leftMaster;

    private final DifferentialDrive drive;

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    Pose2d pose;

    // UPDATE WHEEL TO WHEEL LENGTH
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            Units.inchesToMeters(20));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
            getHeading());

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain() {
        // Talons
        rightMaster = new WPI_TalonFX(Constants.RIGHT_MOTOR_1);
        WPI_TalonFX rightMotor2 = new WPI_TalonFX(Constants.RIGHT_MOTOR_2);
        WPI_TalonFX rightMotor3 = new WPI_TalonFX(Constants.RIGHT_MOTOR_3);

        leftMaster = new WPI_TalonFX(Constants.LEFT_MOTOR_1);
        WPI_TalonFX leftMotor2 = new WPI_TalonFX(Constants.LEFT_MOTOR_2);
        WPI_TalonFX leftMotor3 = new WPI_TalonFX(Constants.LEFT_MOTOR_3);

        rightMotor2.follow(rightMaster);
        rightMotor2.setInverted(InvertType.FollowMaster);
        rightMotor3.follow(rightMaster);
        rightMotor3.setInverted(InvertType.FollowMaster);

        leftMotor2.follow(leftMaster);
        leftMotor3.follow(leftMaster);

        // Same as set invert = false
      TalonFXInvertType m_left_invert = TalonFXInvertType.CounterClockwise;

        //Same as set invert = true
      TalonFXInvertType m_right_invert = TalonFXInvertType.Clockwise;

        leftMaster.setInverted(m_left_invert);
        rightMaster.setInverted(m_right_invert);

        //DifferentialDrive
        drive = new DifferentialDrive(leftMaster, rightMaster);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    // ADJUST GEAR RATIO AND WHEEL DIAMETER!
    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftMaster.getSelectedSensorVelocity() * 600 / 2048 / 7.29 * 2 *
                Math.PI * Units.inchesToMeters(3.0) / 60,
                rightMaster.getSelectedSensorVelocity() * 600 / 2048 / 7.29 *
                2 * Math.PI * Units.inchesToMeters(3.0) / 60);
    }

    @Override
    public void periodic() {
        pose = odometry.update(
                getHeading(),
                leftMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI *
                Units.inchesToMeters(3.0) / 60,
                rightMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI *
                Units.inchesToMeters(3.0) / 60);
    }


    public void driveWithJoysticks(XboxController controller, double speed) {
        drive.tankDrive(
                controller.getRawAxis(Constants.XBOX_LEFT_Y_AXIS),
                controller.getRawAxis(Constants.XBOX_RIGHT_Y_AXIS) * -1);
    }

    public void drive(double left, double right) {
      drive.tankDrive(left, right);
    }
} 
