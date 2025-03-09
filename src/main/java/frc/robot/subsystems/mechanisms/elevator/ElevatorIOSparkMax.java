package frc.robot.subsystems.mechanisms.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private SparkMax m_leftElevator;
  private SparkMax m_rightElevator;
  private SparkClosedLoopController leftElevatorController;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private SparkLimitSwitch bottomLimitSwitch;
  private static boolean zeroed;
  private SparkBaseConfig mLeftConfig;
  private SparkBaseConfig mRightConfig;

  private ElevatorFeedforward feedforward =
      new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, 0.0);

  public ElevatorIOSparkMax() {
    // base config for all motors
    mLeftConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);

    // Create spesific right motor config from base config
    mRightConfig.apply(mLeftConfig);
    mRightConfig.follow(MechanismConstants.leftElevatorId, true);

    m_leftElevator = new SparkMax(MechanismConstants.leftElevatorId, MotorType.kBrushless);
    m_rightElevator = new SparkMax(MechanismConstants.rightElevatorId, MotorType.kBrushless);

    // Adjust left motor closed loop (pid controller) config
    ClosedLoopConfig closedLoopConfig = mLeftConfig.closedLoop;
    closedLoopConfig
        .pid(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false);

    // Adjust left motor max motion position specific config
    MAXMotionConfig maxMotionConfig = mLeftConfig.closedLoop.maxMotion;
    maxMotionConfig
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(ElevatorConstants.allowedClosedLoopError)
        .maxAcceleration(ElevatorConstants.maxAccel)
        .maxVelocity(ElevatorConstants.maxVelo);

    SoftLimitConfig softLimitConfig = mLeftConfig.softLimit;
    softLimitConfig.forwardSoftLimit(ElevatorConstants.maxHeight).forwardSoftLimitEnabled(true);

    // Configure both motors
    m_leftElevator.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevator.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.dutyCycle = m_leftElevator.getAppliedOutput();
    inputs.appliedCurrent = m_leftElevator.getOutputCurrent();
    inputs.appliedVolts = m_leftElevator.getBusVoltage() * m_leftElevator.getAppliedOutput();
    inputs.elevatorVelo = leftEncoder.getVelocity();
    inputs.elevatorPos = leftEncoder.getPosition();
    if (bottomLimitSwitch.isPressed()) {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      zeroed = true;
    }

    inputs.zeroed = zeroed;
  }

  public void requestElevatorPosition(double targetPosition) {
    if (zeroed) {
      leftElevatorController.setReference(
          targetPosition,
          ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0,
          feedforward.calculate(leftEncoder.getVelocity()),
          ArbFFUnits.kVoltage);
    } else {
      leftElevatorController.setReference(-1, ControlType.kVoltage);
    }
  }

  /**
   * Drive motors in voltage control mode
   *
   * @param voltage Voltage to drive motors at
   */
  public void voltageControl(double voltage) {
    voltage = voltage + ElevatorConstants.kg;
    // clamp to -12, 12 volts
    voltage = Math.max(-12.0, Math.min(voltage, 12.0));
    leftElevatorController.setReference(voltage, ControlType.kVoltage);
  }

  public void stop() {
    m_leftElevator.stopMotor();
    m_rightElevator.stopMotor();
  }
}
