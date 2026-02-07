// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  
  private final SparkMax motor1, motor2;
  private final SparkMaxConfig motor1Config, motor2Config;
  public Climber() {
    motor1 = new SparkMax(ClimberConstants.MOTOR1_ID, MotorType.kBrushless);
    motor1Config = new SparkMaxConfig();

    motor1Config
      .inverted(ClimberConstants.INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ClimberConstants.SMARTCURRENTLIMIT);

    motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor2 = new SparkMax(ClimberConstants.MOTOR2_ID, MotorType.kBrushless);
    motor2Config = new SparkMaxConfig();

    motor2Config
      .follow(motor1,true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ClimberConstants.SMARTCURRENTLIMIT);

    motor2.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimberSpeed(double speed){
    motor1.set(speed);
  }

  public Command runClimber(double speed){
    return new InstantCommand(() -> setClimberSpeed(speed));
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
