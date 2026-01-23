// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  public final SparkFlex motor1, motor2;
  public final SparkFlexConfig motor1Config,motor2Config;
  public Intake() {
    motor1 = new SparkFlex(IntakeConstants.MOTOR1_ID, MotorType.kBrushless);
    motor1Config = new SparkFlexConfig();

    motor1Config
      .inverted(IntakeConstants.MOTOR1_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(IntakeConstants.SMARTCURRENTLIMIT);
    
    motor1Config.signals
      .faultsPeriodMs(IntakeConstants.FAULTSPERIOD)
      .outputCurrentPeriodMs(IntakeConstants.INDEXER_OUTPUT_CURRENT_PERIOD);

    motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor2 = new SparkFlex(IntakeConstants.MOTOR2_ID, MotorType.kBrushless);
    motor2Config = new SparkFlexConfig();

    motor2Config
      .follow(IntakeConstants.MOTOR1_ID, IntakeConstants.MOTOR2_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(IntakeConstants.SMARTCURRENTLIMIT);

    motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setSpeed(double speed){
    motor1.set(speed);
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
