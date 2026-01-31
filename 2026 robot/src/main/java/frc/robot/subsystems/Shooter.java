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


public class Shooter extends SubsystemBase {
  private final SparkFlex topRollerMotor1, bottomRollerMotor1, bottomRollerMotor2; 
  private final SparkMax indexer;
  private final SparkFlexConfig topRollerMotor1Config, bottomRollerMotor1Config, bottomRollerMotor2Config;
  private final SparkMaxConfig indexerConfig;
  private final SparkClosedLoopController topRollerPID, bottomRollerPID, bottomRoller2PID,indexerPID;
  private SimpleMotorFeedforward topRollerfeedforward, bottomRollerfeedforward, bottomRoller2feedforward,indexerFeedForward;
  private final RelativeEncoder topRollerEncoder, bottomRollerEncoder;

  private double topRollerSetPoint, bottomRollerSetPoint;

  public Shooter() {
    topRollerMotor1 = new SparkFlex(ShooterConstants.TOPROLLER_ID, MotorType.kBrushless);
    topRollerMotor1Config = new SparkFlexConfig();
    topRollerPID = topRollerMotor1.getClosedLoopController();
    topRollerEncoder = topRollerMotor1.getEncoder();

    topRollerMotor1Config
      .inverted(ShooterConstants.TOPROLLERINVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.TOPROLLER_SMARTCURRENTLIMIT);
    
    topRollerMotor1Config.signals
      .primaryEncoderPositionAlwaysOn(ShooterConstants.SHOOTER_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderVelocityAlwaysOn(ShooterConstants.SHOOTER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.SHOOTER_VELOCITY_PERIOD)
      .outputCurrentPeriodMs(ShooterConstants.SHOOTEROUTPUTCURRENT_PERIOD);

    topRollerMotor1Config.closedLoop
      .pid(ShooterConstants.TOPROLLER_KP
        ,ShooterConstants.TOPROLLER_KI
        ,ShooterConstants.TOPROLLER_KD)
      .outputRange(0, 1);
    
    topRollerMotor1.configure(topRollerMotor1Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    topRollerfeedforward = new SimpleMotorFeedforward(ShooterConstants.TOPROLLER_KS, ShooterConstants.TOPROLLER_KV, ShooterConstants.TOPROLLER_KA);

    topRollerSetPoint = ShooterConstants.TOPROLLER_IDLE_SPEED;

    // topRollerMotor2 = new SparkFlex(ShooterConstants.TOPROLLERMOTOR2_ID, MotorType.kBrushless);
    // topRollerMotor2Config = new SparkFlexConfig();

    // topRollerMotor2Config
    //   .follow(topRollerMotor1, ShooterConstants.TOPROLLERMOTOR2INVERTED)
    //   .idleMode(IdleMode.kCoast)
    //   .smartCurrentLimit(ShooterConstants.TOPROLLER_SMARTCURRENTLIMIT);
    
    //topRollerMotor2.configure(topRollerMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


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
    bottomRoller2PID = bottomRollerMotor2.getClosedLoopController();

    bottomRollerMotor2Config
      .inverted( ShooterConstants.BOTTOMROLLER2INVERTED)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.BOTTOMROLLER_SMARTCURRENTLIMIT);
    bottomRollerMotor2Config.signals
      .primaryEncoderPositionAlwaysOn(ShooterConstants.SHOOTER_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderVelocityAlwaysOn(ShooterConstants.SHOOTER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.SHOOTER_VELOCITY_PERIOD)
      .outputCurrentPeriodMs(ShooterConstants.SHOOTEROUTPUTCURRENT_PERIOD);
    bottomRollerMotor2Config.closedLoop
      .pid(ShooterConstants.BOTTOMROLLER2_KP
        ,ShooterConstants.BOTTOMROLLER2_KI
        ,ShooterConstants.BOTTOMROLLER2_KD)
      .outputRange(0, 1);

    bottomRollerMotor2.configure(bottomRollerMotor2Config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    bottomRollerfeedforward = new SimpleMotorFeedforward(ShooterConstants.BOTTOMROLLER_KS, ShooterConstants.BOTTOMROLLER_KV, ShooterConstants.BOTTOMROLLER_KA);
    bottomRoller2feedforward = new SimpleMotorFeedforward(ShooterConstants.BOTTOMROLLER2_KS, ShooterConstants.BOTTOMROLLER2_KV, ShooterConstants.BOTTOMROLLER2_KA);


    bottomRollerSetPoint = ShooterConstants.BOTTOMROLLER_IDLE_SPEED;

    setShooterSpeeds(topRollerSetPoint, bottomRollerSetPoint);

    indexer = new SparkMax(ShooterConstants.INDEXER_ID, MotorType.kBrushless);
    indexerConfig = new SparkMaxConfig();
    indexerPID = indexer.getClosedLoopController();
    

    indexerConfig
      .inverted(ShooterConstants.INDEXER_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ShooterConstants.INDEXER_SMARTCURRENTLIMIT);
    indexerConfig.signals
      .faultsPeriodMs(ShooterConstants.INDEXER_FAULTS_PERIOD_MS)
      .primaryEncoderPositionPeriodMs(ShooterConstants.INDEXER_POSITION_PERIOD)
      .primaryEncoderVelocityPeriodMs(ShooterConstants.INDEXER_VELOCITY_PERIOD)
      .outputCurrentPeriodMs(ShooterConstants.INDEXER_OUTPUT_CURRENT_PERIOD);
     indexerConfig.closedLoop
      .pid(ShooterConstants.INDEXER_KP
        ,ShooterConstants.INDEXER_KP
        ,ShooterConstants.INDEXER_KP)
      .outputRange(0, 1);
    
    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    indexerFeedForward = new SimpleMotorFeedforward(ShooterConstants.INDEXER_KS, ShooterConstants.INDEXER_KV);
   
    setShooterSpeeds(400,400);

    
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

    bottomRoller2PID.setSetpoint(
      bottomRollerRPM,
      ControlType.kVelocity, 
      ClosedLoopSlot.kSlot0,
      bottomRoller2feedforward.calculate(bottomRollerRPM));
    


    topRollerSetPoint = topRollerRPM;
    bottomRollerSetPoint = bottomRollerRPM;
  }
  public void setShooterExitVelocity(double velocity){
    double rpm = velocity * ShooterConstants.VELOCITY_to_RPM_INTERPOLATIONSLOPE  + ShooterConstants.VELOCITY_to_RPM_INTERPOLATIONINTERCEPT;
    //use an interpolation to find what rpms will give the desired velocity
    //setShooterSpeeds(rpm,rpm);
    setShooterSpeeds(rpm, rpm);
  }
  public boolean shooterAtSpeed(){
    return Math.abs(topRollerSetPoint - topRollerEncoder.getVelocity()) <= ShooterConstants.SHOOTERTOLERANCE &&
    Math.abs (bottomRollerSetPoint - bottomRollerEncoder.getVelocity()) <= ShooterConstants.SHOOTERTOLERANCE;
  } 

  public void setTopRollerRaw(double speed){
    topRollerMotor1.set(speed);
  }
  public void setSpeedsSmartDashboard(){
    setShooterSpeeds(SmartDashboard.getNumber("setTopRoller", 0),SmartDashboard.getNumber("setBottomRoller", 0));
    // topRollerMotor1.setVoltage(SmartDashboard.getNumber("setTopRoller", 0));
    // bottomRollerMotor1.setVoltage(SmartDashboard.getNumber("setBottomRoller", 0));
  }
  public void setBottomRollerRaw(double speed){
    bottomRollerMotor1.set(speed);
  }

  public void setIndexerSpeed(double speed){
    indexer.set(speed);
  }

  public void setIndexerPID(double rpm){
    indexerPID.setSetpoint(
      rpm,
      ControlType.kVelocity, 
      ClosedLoopSlot.kSlot0,
      indexerFeedForward.calculate(rpm));

  }
  public double getTopRollerCurrent(){
    return topRollerMotor1.getOutputCurrent();
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

    topRollerMotor1Config.closedLoop.pid(
    SmartDashboard.getNumber("topRollerKP",0),
    SmartDashboard.getNumber("topRollerKI",0),
    SmartDashboard.getNumber("topRollerKD",0));
    topRollerMotor1.configure(topRollerMotor1Config,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

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
    SmartDashboard.putNumber("bottomRoller2RPM",bottomRollerMotor2.getEncoder().getVelocity());
    SmartDashboard.putNumber("topRollerVoltage", topRollerMotor1.getBusVoltage());
    SmartDashboard.putNumber("bottomRollerVoltage", bottomRollerMotor1.getBusVoltage());

    SmartDashboard.putNumber("indexerSpeed",indexer.getEncoder().getVelocity());
    
    SmartDashboard.putNumber("topRoller1ControlEffort", topRollerMotor1.getAppliedOutput());
    SmartDashboard.putNumber("bottomRoller1ControlEffort", bottomRollerMotor1.getAppliedOutput());
    SmartDashboard.putNumber("bottomRoller2ControlEffort", bottomRollerMotor2.getAppliedOutput());

    SmartDashboard.putNumber("shooterTargetRPM", topRollerSetPoint);
    SmartDashboard.putBoolean("shooteAtSpped", shooterAtSpeed());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
