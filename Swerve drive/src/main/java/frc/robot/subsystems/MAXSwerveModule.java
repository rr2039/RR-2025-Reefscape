// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final TalonFX m_drivingTalonFX;
  private final SparkMax m_turningSparkMax;

  //private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxConfig m_SparkMaxConfig;

  //private final SparkPIDController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  private final ClosedLoopConfig m_turningPIDconfig;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    //m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_drivingTalonFX = new TalonFX(drivingCANId);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    m_turningPIDconfig = new ClosedLoopConfig();
    m_SparkMaxConfig = new SparkMaxConfig();

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    //m_drivingSparkMax.restoreFactoryDefaults();
    //m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    //m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    //m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.

    
   // m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    //m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.


    //m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDconfig.positionWrappingEnabled(true);
    m_turningPIDconfig.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDconfig.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ModuleConstants.kDrivingP;
    slot0Configs.kI = ModuleConstants.kDrivingI;
    slot0Configs.kD = ModuleConstants.kDrivingD;
    slot0Configs.kV = ModuleConstants.kDrivingFF;

    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = 100;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 60;
    currentLimits.SupplyCurrentLimitEnable = true;

    m_drivingTalonFX.getConfigurator().apply(slot0Configs);
    m_drivingTalonFX.getConfigurator().apply(currentLimits);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDconfig.pid(ModuleConstants.kTurningP,ModuleConstants.kTurningI,ModuleConstants.kTurningD);
    m_turningPIDconfig.velocityFF(ModuleConstants.kTurningFF);
    m_turningPIDconfig.outputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingTalonFX.setNeutralMode(NeutralModeValue.Brake);
    m_SparkMaxConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_turningSparkMax.configure(m_SparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);


    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingTalonFX.setPosition(0);
  }

  // Revs/second to Revs/min
  public double getDriveVelocity() {
    return (m_drivingTalonFX.getVelocity().getValueAsDouble()) * ModuleConstants.kDrivingEncoderVelocityFactor;
  }


  private double getDrivePosition() {
    return m_drivingTalonFX.getPosition().getValueAsDouble() * ModuleConstants.kDrivingEncoderPositionFactor;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(getDriveVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        getDrivePosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
    new Rotation2d(m_turningEncoder.getPosition()));
    /*SwerveModuleState optimizedDesiredState = new SwerveModuleState();
    optimizedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));*/

    double test = (optimizedDesiredState.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters * Math.PI)) * ModuleConstants.kDrivingMotorReduction;  
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    //System.out.println(optimizedDesiredState.speedMetersPerSecond);
    //System.out.println(test);
    VelocityDutyCycle optimizedSpeed = new VelocityDutyCycle(test);
    //System.out.println(optimizedDesiredState.speedMetersPerSecond);
    optimizedSpeed.EnableFOC = true;
    m_drivingTalonFX.setControl(optimizedSpeed);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalonFX.setPosition(0);
  }
}
