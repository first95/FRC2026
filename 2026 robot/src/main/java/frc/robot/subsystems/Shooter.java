// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;

import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond; 

public class Shooter extends SubsystemBase {
  private final SparkFlex topRoller, bottomRoller;
  private final SparkFlexConfig topRollerConfig, bottomRollerConfig;
  private final SparkClosedLoopController topRollerPID, bottomRollerPID;
  private final SimpleMotorFeedforward topRollerfeedforward, bottomRollerfeedforward;

  private final SysIdRoutine topRollerCharacterizer;

  public Shooter() {
    topRoller = new SparkFlex(ShooterConstants.TOPROLLER_ID, MotorType.kBrushless);
    topRollerConfig = new SparkFlexConfig();
    topRollerPID = topRoller.getClosedLoopController();

    topRollerConfig
      .inverted(ShooterConstants.TOPROLLERINVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.TOPROLLER_SMARTCURRENTLIMIT);
    
    topRollerConfig.signals
      .primaryEncoderPositionAlwaysOn(ShooterConstants.PRIMARY_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderVelocityAlwaysOn(ShooterConstants.PRIMARY_ENCODER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.PRIMARY_ENCODER_VELOCITY_PERIOD);

    topRollerConfig.closedLoop
      .pid(ShooterConstants.TOPROLLER_KP
        ,ShooterConstants.TOPROLLER_KI
        ,ShooterConstants.TOPROLLER_KD);
    
    topRoller.configure(topRollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    topRollerfeedforward = new SimpleMotorFeedforward(ShooterConstants.TOPROLLER_KS, ShooterConstants.TOPROLLER_KV, ShooterConstants.TOPROLLER_KA);

    bottomRoller = new SparkFlex(ShooterConstants.BOTTOMROLLER_ID, MotorType.kBrushless);
    bottomRollerConfig = new SparkFlexConfig();
    bottomRollerPID = bottomRoller.getClosedLoopController();

    bottomRollerConfig
      .inverted(ShooterConstants.BOTTOMROLLERINVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.BOTTOMROLLER_SMARTCURRENTLIMIT);
    
    bottomRollerConfig.signals
      .primaryEncoderPositionAlwaysOn(ShooterConstants.PRIMARY_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderVelocityAlwaysOn(ShooterConstants.PRIMARY_ENCODER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.PRIMARY_ENCODER_VELOCITY_PERIOD);

    bottomRollerConfig.closedLoop
      .pid(ShooterConstants.BOTTOMROLLER_KP
        ,ShooterConstants.BOTTOMROLLER_KI
        ,ShooterConstants.BOTTOMROLLER_KD);
    
    bottomRoller.configure(bottomRollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    bottomRollerfeedforward = new SimpleMotorFeedforward(ShooterConstants.BOTTOMROLLER_KS, ShooterConstants.BOTTOMROLLER_KV, ShooterConstants.BOTTOMROLLER_KA);

    topRollerCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              topRoller.setVoltage(voltage.in(Volts));
            },
            log -> {
              log.motor("topRollerofShooter")
                  .voltage(Volts.of(topRoller.getAppliedOutput() * topRoller.getBusVoltage()))
                  .angularPosition(Rotations.of(topRoller.getEncoder().getPosition()))
                  .angularVelocity(RotationsPerSecond.of(topRoller.getEncoder().getVelocity()*60));
            },
            this));
  }

  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
