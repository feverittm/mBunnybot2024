// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  public final double kMaxSpeed = 3.0; // meters per second
  public final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private final CANSparkMax m_leftLeader = new CANSparkMax(DrivetrainConstants.leftLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DrivetrainConstants.leftFollowerChannel,
      MotorType.kBrushless);
  private final CANSparkMax m_rightLeader = new CANSparkMax(DrivetrainConstants.rightLeaderChannel,
      MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DrivetrainConstants.rightFollowerChannel,
      MotorType.kBrushless);

  private final Encoder m_leftEncoder = new Encoder(DrivetrainConstants.leftEncoderChannels[0],
      DrivetrainConstants.leftEncoderChannels[1]);
  private final Encoder m_rightEncoder = new Encoder(DrivetrainConstants.rightEncoderChannels[0],
      DrivetrainConstants.rightEncoderChannels[1]);

  private AHRS ahrs;

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  private boolean m_driveDirection = true;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      ahrs = new AHRS(Port.kUSB);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    ahrs.reset();

    m_leftLeader.restoreFactoryDefaults();
    m_rightLeader.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFollower.setInverted(true);
    m_rightLeader.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_odometry = new DifferentialDriveOdometry(
        ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    m_driveDirection = true;
  }

  /**
   * Set driving direction
   * @param boolean direction
   * true: Regular forward with intake/chute forward.
   * false: Reversed.  Totermover is set as the 'front' of the robot.
   */
  public void setDriveDirection(boolean direction) {
    m_driveDirection = direction;
    SmartDashboard.putBoolean("Drive Direction", m_driveDirection);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftLeader.setVoltage(m_driveDirection ? leftOutput + leftFeedforward : -rightOutput);
    m_rightLeader.setVoltage(m_driveDirection ? rightOutput + rightFeedforward : -leftOutput);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public Command setDriveForward() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setDriveDirection(true);
        });
  }

  public Command setDriveReverse() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setDriveDirection(false);;
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.updateOdometry();
  }
}
