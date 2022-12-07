// Copyright (c) Darius and other Darius contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file and DARI lisence in the root directory of this
// project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Ctake;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.MotorController;
import java.util.Arrays;
import java.util.List;

public class DriveTrain extends SubsystemBase {
  // Local Talon Variables
  private final WPI_TalonFX rightMaster;
  private final WPI_TalonFX leftMaster;

  private final WPI_TalonFX rightMotor2;
  private final WPI_TalonFX rightMotor3;
  private final WPI_TalonFX leftMotor2;
  private final WPI_TalonFX leftMotor3;

  private final List<WPI_TalonFX> allMotors;

  private final DoubleSolenoid shifter;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry m_Odometry;
  private final Field2d field;

  private final MotorController unitConverter;

  private final StatorCurrentLimitConfiguration statorLimitConfig =
      new StatorCurrentLimitConfiguration(true, 40, 70, 2);

  private final SupplyCurrentLimitConfiguration supplyLimitConfig =
      new SupplyCurrentLimitConfiguration(true, 40, 60, 4);

  private final NeutralMode neutralMode = NeutralMode.Brake;

  private double rampRate;
  private final int timeoutMs;

  // Simulation (All of these encoders, including the non-sim one are just made for ease of access
  // and not real)
  private final Encoder leftEncoder, rightEncoder;
  private EncoderSim leftEncoderSim, rightEncoderSim;

  private final ADXRS450_Gyro m_gyro;
  private ADXRS450_GyroSim m_GyroSim;

  public DifferentialDrivetrainSim driveSim;

  private Field2d fieldSim;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Talons
    rightMaster = new WPI_TalonFX(DriveConstants.RIGHT_MASTER);
    rightMotor2 = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_2);
    rightMotor3 = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_3);
    leftMaster = new WPI_TalonFX(DriveConstants.LEFT_MASTER);
    leftMotor2 = new WPI_TalonFX(DriveConstants.LEFT_MOTOR_2);
    leftMotor3 = new WPI_TalonFX(DriveConstants.LEFT_MOTOR_3);

    allMotors =
        Arrays.asList(rightMaster, rightMotor2, rightMotor3, leftMaster, leftMotor2, leftMotor3);

    // **********************************************
    // ************** Falcon Configs ****************
    // **********************************************

    // TODO: Adjust rampRate
    rampRate = 0.25;
    timeoutMs = 15;

    allMotors.forEach(
        motor -> {
          // Factory Resets all TalonFX
          motor.configFactoryDefault();
          // Sets Motor to Brake/Coast
          motor.setNeutralMode(neutralMode);
          // Current limit to prevent breaker tripping. Approx at 150% of rated
          // current supply.
          motor.configSupplyCurrentLimit(supplyLimitConfig);
          motor.configStatorCurrentLimit(statorLimitConfig);
          // Ramping motor output to prevent instantaneous directional changes
          // (Values need testing)
          motor.configOpenloopRamp(rampRate, timeoutMs);
        });

    // Same as set invert = true
    TalonFXInvertType leftInvert = TalonFXInvertType.Clockwise;

    // Same as set invert = false
    TalonFXInvertType rightInvert = TalonFXInvertType.CounterClockwise;

    leftMaster.setInverted(leftInvert);
    rightMaster.setInverted(rightInvert);

    rightMotor2.follow(rightMaster);
    rightMotor2.setInverted(InvertType.FollowMaster);
    rightMotor3.follow(rightMaster);
    rightMotor3.setInverted(InvertType.FollowMaster);

    leftMotor2.follow(leftMaster);
    leftMotor2.setInverted(InvertType.FollowMaster);
    leftMotor3.follow(leftMaster);
    leftMotor3.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, timeoutMs);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, timeoutMs);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);

    // Simulation
    leftEncoder = new Encoder(0, 1, true);
    rightEncoder = new Encoder(2, 3, false);

    leftEncoder.reset();
    rightEncoder.reset();

    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDPP);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDPP);

    m_gyro = new ADXRS450_Gyro();

    if (RobotBase.isSimulation()) {
      driveSim =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
      m_GyroSim = new ADXRS450_GyroSim(m_gyro);

      fieldSim = new Field2d();
      SmartDashboard.putData("Field Sim", fieldSim);
    }

    m_Odometry = new DifferentialDriveOdometry(getHeading());
    field = new Field2d();
    resetOdometry(Constants.Odometry.STARTING_POSITION);

    unitConverter = new MotorController(leftMaster);

    shifter =
        new DoubleSolenoid(
            Ctake.MODULE_NUMBER,
            PneumaticsModuleType.REVPH,
            DriveConstants.SHIFTER_FORWARD_CHANNEL,
            DriveConstants.SHIFTER_REVERSE_CHANNEL);
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  public void drive(double left, double right) {
    drive.tankDrive(left, right);
    // TODO: see if this works if not uncomment motor safety
    drive.feed();
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void stopMotors() {
    drive.stopMotor();
  }

  public void shiftGears() {
    Value oppValue = shifter.get() == Value.kForward ? Value.kReverse : Value.kForward;
    shifter.set(oppValue);
  }

  // *****************************************
  // ************** Encoders *****************
  // *****************************************

  public double getLeftDistance() {
    return unitConverter.nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition());
  }

  public double getRightDistance() {
    return unitConverter.nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition());
  }

  public double getRawDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public double getLeftVelocity() {
    return unitConverter.nativeUnitsToVelocity(leftMaster.getSelectedSensorVelocity());
  }

  public double getRightVelocity() {
    return unitConverter.nativeUnitsToVelocity(rightMaster.getSelectedSensorVelocity());
  }

  public double getVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2.0;
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  // Returns the robots angle as a double
  public double getRawGyroAngle() {
    return m_gyro.getAngle() % 360;
  }

  // Return robot heading in degrees, from -180 to 180
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  // ********************************************
  // ************ Odometry Functions ************
  // ********************************************

  public void updateOdometry() {
    m_Odometry.update(getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public Rotation2d getRotation2d() {

    // TODO: Test if it needs to be negative or nah

    return m_gyro.getRotation2d();
  }

  public void resetOdometry(Pose2d pose2d) {
    m_gyro.reset();
    resetEncoders();
    m_Odometry.resetPosition(pose2d, getHeading());
  }

  public Field2d getField() {
    return field;
  }

  public Pose2d getPose() {
    updateOdometry();
    return m_Odometry.getPoseMeters();
  }

  public void reset() {
    resetOdometry(getPose());
  }

  // *****************************************
  // ************** Voltage ******************
  // *****************************************

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return driveSim.getCurrentDrawAmps();
  }

  @Override
  public void periodic() {
    updateOdometry();
    field.setRobotPose(getPose());
    fieldSim.setRobotPose(getPose());
    ;

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(
        leftMaster.get() * RobotController.getBatteryVoltage(),
        rightMaster.get() * RobotController.getBatteryVoltage());
    driveSim.update(0.02);

    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    m_GyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Angle", this::getRawGyroAngle, null);
    builder.addDoubleProperty(
        "ramp rate",
        () -> rampRate,
        r -> {
          System.out.println("setting ramp rate");
          rampRate = r;
          allMotors.forEach(motor -> motor.configOpenloopRamp(rampRate, timeoutMs));
        });
  }
}
