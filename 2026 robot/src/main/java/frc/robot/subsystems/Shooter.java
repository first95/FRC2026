// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;

import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond; 

public class Shooter extends SubsystemBase {
  private final SparkFlex topRoller, bottomRollerMotor1, bottomRollerMotor2; 
  private final SparkMax indexer;
  private final SparkFlexConfig topRollerConfig, bottomRollerMotor1Config, bottomRollerMotor2Config;
  private final SparkMaxConfig indexerConfig;
  private final SparkClosedLoopController topRollerPID, bottomRollerPID;
  private SimpleMotorFeedforward topRollerfeedforward, bottomRollerfeedforward;
  private final RelativeEncoder topRollerEncoder, bottomRollerEncoder;

  private double topRollerSetPoint, bottomRollerSetPoint;

  private final SysIdRoutine topRollerCharacterizer,bottomRollerCharacterizer;

  public Shooter() {
    topRoller = new SparkFlex(ShooterConstants.TOPROLLER_ID, MotorType.kBrushless);
    topRollerConfig = new SparkFlexConfig();
    topRollerPID = topRoller.getClosedLoopController();
    topRollerEncoder = topRoller.getEncoder();

    topRollerConfig
      .inverted(ShooterConstants.TOPROLLERINVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.TOPROLLER_SMARTCURRENTLIMIT);
    
    topRollerConfig.signals
      .primaryEncoderPositionAlwaysOn(ShooterConstants.SHOOTER_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderVelocityAlwaysOn(ShooterConstants.SHOOTER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.SHOOTER_VELOCITY_PERIOD)
      .outputCurrentPeriodMs(ShooterConstants.SHOOTEROUTPUTCURRENT_PERIOD);

    topRollerConfig.closedLoop
      .pid(ShooterConstants.TOPROLLER_KP
        ,ShooterConstants.TOPROLLER_KI
        ,ShooterConstants.TOPROLLER_KD)
      .outputRange(0, 1);;
    
    topRoller.configure(topRollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    topRollerfeedforward = new SimpleMotorFeedforward(ShooterConstants.TOPROLLER_KS, ShooterConstants.TOPROLLER_KV, ShooterConstants.TOPROLLER_KA);

    topRollerSetPoint = ShooterConstants.TOPROLLER_IDLE_SPEED;

    bottomRollerMotor1 = new SparkFlex(ShooterConstants.BOTTOMROLLER_ID, MotorType.kBrushless);
    bottomRollerMotor1Config = new SparkFlexConfig();
    bottomRollerPID = bottomRollerMotor1.getClosedLoopController();
    bottomRollerEncoder = bottomRollerMotor1.getEncoder();

    bottomRollerMotor1Config
      .inverted(ShooterConstants.BOTTOMROLLERINVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.BOTTOMROLLER_SMARTCURRENTLIMIT);
    
    bottomRollerMotor1Config.signals
      .primaryEncoderPositionAlwaysOn(ShooterConstants.SHOOTER_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderVelocityAlwaysOn(ShooterConstants.SHOOTER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.SHOOTER_VELOCITY_PERIOD)
      .outputCurrentPeriodMs(ShooterConstants.SHOOTEROUTPUTCURRENT_PERIOD);

    bottomRollerMotor1Config.closedLoop
      .pid(ShooterConstants.BOTTOMROLLER_KP
        ,ShooterConstants.BOTTOMROLLER_KI
        ,ShooterConstants.BOTTOMROLLER_KD)
      .outputRange(0, 1);
    
    bottomRollerMotor1.configure(bottomRollerMotor1Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    bottomRollerMotor2 = new SparkFlex(ShooterConstants.BOTTOMROLLERMOTOR2_ID, MotorType.kBrushless);
    bottomRollerMotor2Config = new SparkFlexConfig();

    bottomRollerMotor2Config
      .follow(bottomRollerMotor1, ShooterConstants.BOTTOMROLLER2INVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.BOTTOMROLLER_SMARTCURRENTLIMIT);
    

    bottomRollerfeedforward = new SimpleMotorFeedforward(ShooterConstants.BOTTOMROLLER_KS, ShooterConstants.BOTTOMROLLER_KV, ShooterConstants.BOTTOMROLLER_KA);

    bottomRollerSetPoint = ShooterConstants.BOTTOMROLLER_IDLE_SPEED;

    setShooterSpeeds(topRollerSetPoint, bottomRollerSetPoint);

    indexer = new SparkMax(ShooterConstants.INDEXER_ID, MotorType.kBrushless);
    indexerConfig = new SparkMaxConfig();

    indexerConfig
      .inverted(ShooterConstants.INDEXER_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ShooterConstants.INDEXER_SMARTCURRENTLIMIT);
    indexerConfig.signals
      .faultsPeriodMs(ShooterConstants.INDEXER_FAULTS_PERIOD_MS)
      .primaryEncoderPositionPeriodMs(ShooterConstants.INDEXER_POSITION_PERIOD)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.INDEXER_VELOCITY_PERIOD)
      .outputCurrentPeriodMs(ShooterConstants.INDEXER_OUTPUT_CURRENT_PERIOD);
    
    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexer.set(0.3);
    setShooterSpeeds(1900,1900);

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
     bottomRollerCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              bottomRollerMotor1.setVoltage(voltage.in(Volts));
              bottomRollerMotor2.setVoltage(voltage.in(Volts));
            },
            log -> {
              log.motor("bottomRollerofShooter")
                  .voltage(Volts.of(topRoller.getAppliedOutput() * topRoller.getBusVoltage()))
                  .angularPosition(Rotations.of(topRoller.getEncoder().getPosition()))
                  .angularVelocity(RotationsPerSecond.of(topRoller.getEncoder().getVelocity()*60));
            },
            this));
  }


  public void setShooterSpeeds(double topRollerRPM, double bottomRollerRPM){

    topRollerPID.setSetpoint(
      topRollerRPM, 
      ControlType.kVelocity, 
      ClosedLoopSlot.kSlot0,
      topRollerfeedforward.calculate(topRollerRPM));
    
    bottomRollerPID.setSetpoint(
      bottomRollerRPM,
      ControlType.kVelocity, 
      ClosedLoopSlot.kSlot0,
      bottomRollerfeedforward.calculate(bottomRollerRPM));

    topRollerSetPoint = topRollerRPM;
    bottomRollerSetPoint = bottomRollerRPM;
  }
  public void setShooterExitVelocity(double velocity){
    
    //use an interpolation to find what rpms will give the desired velocity
    setShooterSpeeds(0,0);
  }
  public boolean shooterAtSpeed(){
    return Math.abs(topRollerSetPoint - topRollerEncoder.getVelocity()) <= ShooterConstants.SHOOTERTOLERANCE &&
    Math.abs (bottomRollerSetPoint - bottomRollerEncoder.getVelocity()) <= ShooterConstants.SHOOTERTOLERANCE;
  } 

  public void setTopRollerRaw(double speed){
    topRoller.set(speed);
  }
  public void setSpeedsSmartDashboard(){
    setShooterSpeeds(SmartDashboard.getNumber("setTopRoller", 0),SmartDashboard.getNumber("setBottomRoller", 0));
  }
  public void setBottomRollerRaw(double speed){
    bottomRollerMotor1.set(speed);
  }

  public void setIndexerSpeed(double speed){
    indexer.set(speed);
  }

  public double getTopRollerCurrent(){
    return topRoller.getOutputCurrent();
  }

  public double getBottomRollerCurrent(){
    return bottomRollerMotor1.getOutputCurrent();
  }

  public double getIndexerCurrent(){
    return indexer.getOutputCurrent();
  }

  public void setGains(){
    topRollerfeedforward = new SimpleMotorFeedforward(
    SmartDashboard.getNumber("topRollerKS",0),
    SmartDashboard.getNumber("topRollerKV",0),
    SmartDashboard.getNumber("topRollerKA",0));

    topRollerConfig.closedLoop.pid(
    SmartDashboard.getNumber("topRollerKP",0),
    SmartDashboard.getNumber("topRollerKI",0),
    SmartDashboard.getNumber("topRollerKD",0));
    topRoller.configure(topRollerConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    bottomRollerfeedforward = new SimpleMotorFeedforward(
    SmartDashboard.getNumber("bottomRollerKS",0),
    SmartDashboard.getNumber("bottomRollerKV",0),
    SmartDashboard.getNumber("bottomRollerKA",0));

    bottomRollerMotor1Config.closedLoop.pid(
    SmartDashboard.getNumber("bottomRollerKP",0),
    SmartDashboard.getNumber("bottomRollerKI",0),
    SmartDashboard.getNumber("bottomRollerKD",0));
    bottomRollerMotor1.configure(bottomRollerMotor1Config,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
  }
  

  public Command sysIdDynTopRoller(SysIdRoutine.Direction direction){
    return topRollerCharacterizer.dynamic(direction);
  }
  public Command sysIdDynBottompRoller(SysIdRoutine.Direction direction){
    return bottomRollerCharacterizer.dynamic(direction);
  }
  public Command sysIdQuasiTopRoller(SysIdRoutine.Direction direction){
    return topRollerCharacterizer.quasistatic(direction);
  }
  public Command sysIdQuasiBottompRoller(SysIdRoutine.Direction direction){
    return bottomRollerCharacterizer.quasistatic(direction);
  }
  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("topRollerCurrent",getTopRollerCurrent());
    SmartDashboard.putNumber("bottomRollerCurrent",getBottomRollerCurrent());
    SmartDashboard.putNumber("topRollerRPM",topRollerEncoder.getVelocity());
    SmartDashboard.putNumber("bottomRollerRPM",bottomRollerEncoder.getVelocity());
    SmartDashboard.putNumber("topRollerVoltage", topRoller.getBusVoltage());
    SmartDashboard.putNumber("bottomRollerVoltage", bottomRollerMotor1.getBusVoltage());
    SmartDashboard.putNumber("topRollerControlEffort", topRoller.getAppliedOutput());
    SmartDashboard.putNumber("topRollerControlEffort", bottomRollerMotor1.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
