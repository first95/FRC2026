// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkFlex motor1, motor2;
  private final SparkFlexConfig motor1Config, motor2Config;
  private final SimpleMotorFeedforward intakeFeedForward;
  private final SparkClosedLoopController intakePID;

  private final SparkMax agitator1, agitator2;
  private final SparkMaxConfig agitator1Config ,agitator2Config;

  public Intake() {
    motor1 = new SparkFlex(IntakeConstants.MOTOR1_ID, MotorType.kBrushless);
    motor1Config = new SparkFlexConfig();

    motor1Config
      .inverted(IntakeConstants.MOTOR1_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(IntakeConstants.SMARTCURRENTLIMIT);
    
    motor1Config.signals
      .faultsPeriodMs(IntakeConstants.FAULTSPERIOD)
      .outputCurrentPeriodMs(IntakeConstants.OUTPUT_CURRENT_PERIOD);

    motor1Config.closedLoop
      .pid(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD)
      .outputRange(0, 1);

    intakePID = motor1.getClosedLoopController();
    motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeFeedForward = new SimpleMotorFeedforward(IntakeConstants.KS, IntakeConstants.KV);

    motor2 = new SparkFlex(IntakeConstants.MOTOR2_ID, MotorType.kBrushless);
    motor2Config = new SparkFlexConfig();

    motor2Config
      .follow(IntakeConstants.MOTOR1_ID, IntakeConstants.MOTOR2_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(IntakeConstants.SMARTCURRENTLIMIT);

    motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    agitator1 = new SparkMax(IntakeConstants.AGITATOR1_ID, MotorType.kBrushless);
    agitator1Config = new SparkMaxConfig();

    agitator1Config
      .inverted(IntakeConstants.AGITATORINVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(IntakeConstants.AGITATOR_SMARTCURRENTLIMIT);

    agitator1Config.signals
      .faultsPeriodMs(IntakeConstants.FAULTSPERIOD)
      .outputCurrentPeriodMs(IntakeConstants.OUTPUT_CURRENT_PERIOD);

    agitator1.configure(agitator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    agitator2 = new SparkMax(IntakeConstants.AGITATOR2_ID, MotorType.kBrushless);
    agitator2Config = new SparkMaxConfig();

    agitator2Config
      .inverted(!IntakeConstants.AGITATORINVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(IntakeConstants.AGITATOR_SMARTCURRENTLIMIT);

    agitator2Config.signals
      .faultsPeriodMs(IntakeConstants.FAULTSPERIOD)
      .outputCurrentPeriodMs(IntakeConstants.OUTPUT_CURRENT_PERIOD);

    agitator2.configure(agitator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setRPM(double rpm){
    intakePID.setSetpoint(
      rpm,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0,
      intakeFeedForward.calculate(rpm));
  }
  public void setRawSpeed(double speed){
    motor1.set(speed);
  }

  public void setAgitator1Speed(double speed){
    agitator1.set(speed);
  }
  public void setAgitator2Speed(double speed){
    agitator2.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake1Current", motor1.getOutputCurrent());
    SmartDashboard.putNumber("intake2Current", motor2.getOutputCurrent());
    SmartDashboard.putNumber("intakeRPM", motor1.getEncoder().getVelocity());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
