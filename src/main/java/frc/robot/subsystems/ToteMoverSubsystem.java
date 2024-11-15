// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.LifterConstants;

public class ToteMoverSubsystem extends PIDSubsystem {
  private final CANSparkMax lifterMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final DigitalInput lifterLimitSwitch = new DigitalInput(0);

  private final RelativeEncoder lifterEncoder = lifterMotor.getEncoder();

  private final SimpleMotorFeedforward m_lifterFeedforward = new SimpleMotorFeedforward(LifterConstants.kSVolts,
      LifterConstants.kVVoltSecondsPerRotation);

  /** Creates a new ToteMoverSubsystem. */
  public ToteMoverSubsystem() {
    super(new PIDController(LifterConstants.kP, LifterConstants.kI, LifterConstants.kD));
    getController().setTolerance(LifterConstants.LifterTolerance);

    lifterMotor.setSmartCurrentLimit(10);
    lifterMotor.restoreFactoryDefaults();
    lifterMotor.setInverted(false);

    zeroPosition();
    lifterEncoder.setPositionConversionFactor(LifterConstants.kEncoderDistancePerPulse);

    // setSetpoint(LifterConstants.kLifterTarget);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    lifterMotor.setVoltage(output + m_lifterFeedforward.calculate(setpoint));
  }

  public void setLifterVoltage(double voltage) {
    lifterMotor.setVoltage(voltage);
  }

  public double getLifterCurrent() {
    return lifterMotor.getOutputCurrent();
  }

  public Boolean atLifterLimit() {
    return lifterLimitSwitch.get();
  }

  public void zeroPosition() {
    if (atLifterLimit()) {
      lifterEncoder.setPosition(0);
    }
  }

  @Override
  public double getMeasurement() {
    return lifterEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public double getLifterPosition() {
    return lifterEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
