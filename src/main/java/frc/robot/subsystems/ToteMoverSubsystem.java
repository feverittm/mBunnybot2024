// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import static frc.robot.Constants.LifterConstants;

/**
 * The Totemover is a 4-bar linkage attached to the rear of the robot and
 * designed to extent
 * out the back and drop onto totes and then move them back into the robot.
 * 
 * This subsystem is designed to move the linkage to a specific position by
 * moving a motor to a specific angle.
 * The linkage is powered by a Neo motor on a high-reduction gearbox (100:1).
 * The final gear will not even need
 * to make a full revolution in order for the linkage to move from fully
 * retracted to fully extended.
 * 
 * It is assumed that the encoder measures positive with arm extension. 0->N,
 * where N is fully extended.
 * 
 * There will be 2 control modes: 1 - Manual control of position with limits, 2
 * - Defined positions specified
 * by arm state.
 * 
 * There are 4 distinct (defined) positions:
 * 1 - Stowed. Fully back inside the robot. This is the starting position. Also
 * this position activates
 * a limit switch and when active will reset the motor encoder.
 * 2 - Hold. Position for holding the tote in the robot. Normal inside position.
 * 3 - Prepare. This position is outside the robot bot not low enough to pick up
 * a tote. It is a preperation
 * position for when the robot moves in to pick up a tote.
 * 4 - Capture. This position is the fully extented position. It is expected
 * that this position will
 * drop the clips onto the tote.
 */

public class ToteMoverSubsystem extends PIDSubsystem {
  private final CANSparkMax lifterMotor = new CANSparkMax(LifterConstants.lifterMotorChannel, MotorType.kBrushless);
  private final DigitalInput lifterLimitSwitch = new DigitalInput(LifterConstants.lifterLimitChannel);

  private final AnalogInput toteDistance = new AnalogInput(LifterConstants.toteDistanceSensorPort);

  private final RelativeEncoder lifterEncoder = lifterMotor.getEncoder();

  private final SimpleMotorFeedforward lifterFeedforward = new SimpleMotorFeedforward(LifterConstants.kSVolts,
      LifterConstants.kVVoltSecondsPerRotation);

  public static enum ArmPosition {
    MANUAL, STOWED, HOLD, PREPARE, CAPTURE
  }

  public static enum MoveMode {
    MANUAL, AUTO
  }

  private static ArmPosition ArmState = ArmPosition.STOWED;

  private static MoveMode MoveState = MoveMode.MANUAL;

  /** Creates a new ToteMoverSubsystem. */
  public ToteMoverSubsystem() {
    super(new PIDController(LifterConstants.kP, LifterConstants.kI, LifterConstants.kD));
    getController().setTolerance(LifterConstants.LifterTolerance);

    lifterMotor.setSmartCurrentLimit(10);
    lifterMotor.restoreFactoryDefaults();
    lifterMotor.setInverted(false);

    zeroPosition();
    lifterEncoder.setPositionConversionFactor(LifterConstants.kEncoderDistancePerPulse);

    toteDistance.setOversampleBits(4);


    // setSetpoint(LifterConstants.kLifterTarget);
  }

  public double getArmPositionValue(ArmPosition position) {
    double value = -1;
    switch (position) {
      case MANUAL: // 0
        value = getMeasurement();
      case STOWED: // 1
        value = 0;
        break;
      case HOLD: // 2
        value = LifterConstants.kHoldPosition;
        break;
      case PREPARE: // 3
        value = LifterConstants.kPrepPosition;
        break;
      case CAPTURE: // 4
        value = LifterConstants.kCapturePosition;
        break;
      default:
        return -1;
    }
    return value;
  }

  public void setMotorVoltage(double voltage) {
    if (atLifterLimit()) {
      System.out.println("Monual: Arm Lower Limit Reached");
      voltage = 0;
    } else if (getMeasurement() >= LifterConstants.kCapturePosition) {
      System.out.println("Manual: Arm Extension Limit Reached");
      voltage = 0;
    }
    lifterMotor.setVoltage(voltage);
  }

  /**
   * Move the arm to position with limits.
   * 
   * @param output   Voltage to be applied to motors
   * @param setpoint Position setpoint
   */
  @Override
  public void useOutput(double output, double setpoint) {
    if (getMeasurement() == setpoint) {
      return;
    } else if (atLifterLimit()) {
      System.out.println("PID: Arm Lower Limit Reached");
      setpoint = 0;
      return;
    } else if (getMeasurement() >= LifterConstants.kCapturePosition) {
      System.out.println("PID: Arm Extension Limit Reached");
      setpoint = LifterConstants.kCapturePosition;
      return;
    } else {
      MoveState = MoveMode.AUTO;
      lifterMotor.setVoltage(output + lifterFeedforward.calculate(setpoint));
      return;
    }
  }

  /**
   * Move arm to position given by the mode bit. This is NOT a move to a given
   * position (as in an actual encoder position) but a programmed position or
   * just move manually. Everything is held to limits.
   * We will use the 'hat' switch to control the movement:
   * Up: Manually Extend the arm and save current position. (hat = 0)
   * Right: Move arm to next extended programmed PID position. (hat = 90)
   * Down: Manually retract the arm and save the position. (hat = 180)
   * Left: Move to the previous programmmed position. (hat = 270)
   * 
   * @param mode      Are we used the presets or moving manually?
   * @param direction Extend (True), or Retract (False)
   * 
   *                  Expected Bahaviour:
   *                  - If we are in manual mode then stop the arm and grab the
   *                  current position value.
   *                  them try to determine the closest define position to that
   *                  angle. This was the
   *                  Previous/Next function will work correctly.
   *                  - Pushing the left or right button on the hat switch will
   *                  mode the arm
   *                  to the previous or next position in our list.
   *                  - Manual mode is where the fun is...
   *                  Default action (and action when button is released) is to
   *                  stop the arm
   *                  in it's current position.
   *                  Move action will save the current position, manually (by
   *                  directly setting the
   *                  voltage to the motor) move the arm as long as we have not
   *                  reached the limits.
   * 
   */
  public void moveArmToPosition(MoveMode mode, Boolean direction) {
    if (mode == MoveMode.AUTO) {
      if (MoveState == MoveMode.MANUAL) {
        ArmState = getClosestPosition(direction);
        moveToArmSetpoint(ArmState);
      } 
    } else { // MoveMode.MANUAL
      ArmState = ArmPosition.MANUAL;
      setMotorVoltage(direction ? 0.5 : -0.5);
    }
    MoveState = mode;
  }

  public Command moveToArmSetpoint(ArmPosition position) {
    MoveState = MoveMode.AUTO;
    ArmState = position;
    return this.runOnce(() -> setSetpoint(getArmPositionValue(position)));
  }

  public void stopArmManual(MoveMode mode) {
    if (mode == MoveMode.MANUAL) {
      setMotorVoltage(0);
    }
  }

  /**
   * Find the closest defined position to the current raw arm position.
   * 
   * @return the arm position
   */
  private ArmPosition getClosestPosition(Boolean direction) {
    double currentPosition = getMeasurement();
    for (ArmPosition n : ArmPosition.values()) {
      double armSetpoint = getArmPositionValue(n);
      if (direction = true) {
        if (armSetpoint >= currentPosition) { return n; }
      } else {
          if (armSetpoint <= currentPosition) { return n; }
      }
    }
    return ArmPosition.MANUAL;
  }

  /**
   * Current used for the lifter
   * 
   * @return Current being used by the lifter.
   */
  public double getLifterCurrent() {
    return lifterMotor.getOutputCurrent();
  }

  /**
   * at the top 'STOW' position.
   * 
   * @return
   */
  public Boolean atLifterLimit() {
    return lifterLimitSwitch.get();
  }

  /**
   * should be a trigger to reset encoder.
   */
  public void zeroPosition() {
    if (atLifterLimit()) {
      System.out.println("Reset Arm Encoder Position");
      ArmState = ArmPosition.STOWED;
      lifterEncoder.setPosition(0);
    }
  }

  // Need 4 controls: Move to next

  /**
   * Use this to get arm position
   */
  @Override
  public double getMeasurement() {
    return lifterEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Lifter Arm State", ArmState.name());
    SmartDashboard.putString("Lifter Move State", MoveState.name());
    SmartDashboard.putNumber("Lifter Current", getLifterCurrent());
    SmartDashboard.putNumber("Lifter Position", getMeasurement());
    SmartDashboard.putNumber("Lifter Setpoint", getSetpoint());
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverManualExtend() {
    // implicitly require `this`
    return this.runOnce(() -> moveArmToPosition(MoveMode.MANUAL, true));
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverManualRetract() {
    // implicitly require `this`
    return this.runOnce(() -> moveArmToPosition(MoveMode.MANUAL, false));
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverAutoPrev() {
    return this.runOnce(() -> moveArmToPosition(MoveMode.AUTO, true));
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverAutoNext() {
    return this.runOnce(() -> moveArmToPosition(MoveMode.AUTO, false));
  }

  /** Stop the tote mover. */
  public Command stoptoteMover() {
    return this.runOnce(() -> stopArmManual(MoveState));
  }
}
