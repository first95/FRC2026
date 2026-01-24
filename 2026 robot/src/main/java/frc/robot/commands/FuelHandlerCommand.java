// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;


import java.util.function.BooleanSupplier;

import javax.sound.sampled.TargetDataLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FuelHandlerCommand extends Command {
  
  private final Intake intake;
  private final Shooter shooter;
  private final AbsoluteDrive absdrive;
  private final SwerveBase swerve;

  private BooleanSupplier 
    intakeButtonSupplier, 
    shootButtonSupplier,
    aimButtonSupplier;
  
  private boolean
    intakeButton,
    shootButton,
    aimButton;
  
  private Rotation2d 
  currentRobotHeading, 
  shootingHeading;

  private double 
  shootingVelocity;

  private enum State{
    IDLE, INTAKING, CLEARSHOOTER, PREPINDEXER, AIMING, SHOOTING
  }

  private State currentState;

  public FuelHandlerCommand(
    BooleanSupplier intakeButtonSupplier,
    BooleanSupplier shootButtonSupplier,
    BooleanSupplier aimButtonSupplier,
    Shooter shooter,
    Intake intake,
    AbsoluteDrive absdrive,
    SwerveBase swerve) {

    this.intakeButtonSupplier = intakeButtonSupplier;
    this.shootButtonSupplier = shootButtonSupplier;
    this.aimButtonSupplier = aimButtonSupplier;

    this.shooter = shooter;
    this.intake = intake;
    this.absdrive = absdrive;
    this.swerve = swerve;
    
    addRequirements(shooter);
    addRequirements(intake);
  }



  public void initialize() {
    currentState = State.IDLE;
  }


  public void execute() {

    intakeButton = intakeButtonSupplier.getAsBoolean();
    aimButton = aimButtonSupplier.getAsBoolean();
    shootButton = shootButtonSupplier.getAsBoolean();

    currentRobotHeading = swerve.getPose().getRotation();
    shootingHeading = findStationaryshootingHeading(swerve, Constants.Auton.BLUEHUB);
    shootingVelocity = findStationaryShootingVelocity(swerve, Constants.Auton.BLUEHUB);

    findTimeofFlightofProjectile(swerve,Constants.Auton.BLUEHUB);

    switch(currentState){

      case IDLE: 
        shooter.setIndexerSpeed(0);
        shooter.setShooterExitVelocity(0);
        absdrive.setCenterOfRotation(Constants.Drivebase.CENTEROFROTATION);

        if (intakeButton){
          currentState = State.INTAKING;
        }

        if( shootButton || aimButton){
          currentState = State.AIMING;
        }
      break;

      case INTAKING:
        absdrive.setLocustDriving(true);
        intake.setSpeed(IntakeConstants.INTAKINGSPEED);
        shooter.setTopRollerRaw(ShooterConstants.TOPROLLER_JUGGLING_SPEED);
        shooter.setBottomRollerRaw(ShooterConstants.BOTTOMROLLER_JUGGLING_SPEED);

        if(!intakeButton){
          currentState = State.CLEARSHOOTER;
        }

        if(shootButton || aimButton){
          currentState = State.AIMING;
        }
      break;

      case CLEARSHOOTER:
        absdrive.setLocustDriving(false);
        shooter.setIndexerSpeed(ShooterConstants.INDEXER_CLEARSHOOTERSPEED);
        shooter.setTopRollerRaw(ShooterConstants.TOPROLLER_JUGGLING_SPEED);
        shooter.setBottomRollerRaw(ShooterConstants.BOTTOMROLLER_JUGGLING_SPEED);

        if (shooter.getTopRollerCurrent() <= ShooterConstants.TOPROLLER_CLEARSHOOTER_CURRENT_THRESHOLD && shooter.getBottomRollerCurrent() <= ShooterConstants.TOPROLLER_CLEARSHOOTER_CURRENT_THRESHOLD){
          currentState = State.IDLE;
        } 

        if (intakeButton){
          currentState = State.INTAKING;
        }
      break;

      case PREPINDEXER:
        shooter.setIndexerSpeed(ShooterConstants.INDEXERPREPSPEED);
        shooter.setShooterExitVelocity(0);

        if(shooter.getIndexerCurrent() >= ShooterConstants.INDEXER_PREPINDEXER_CURRENT_THRESHOLD){
          currentState = State.IDLE;
        }

        if(intakeButton){
          currentState = State.INTAKING;
        }

        if(shootButton || aimButton){
          currentState = State.AIMING;
        }
      break;

      case AIMING:
        shooter.setShooterExitVelocity(shootingVelocity);
        shooter.setIndexerSpeed(0);

        absdrive.setLocustDriving(false);
        absdrive.setCenterOfRotation(ShooterConstants.SHOOTERLOCATION.toTranslation2d());
        absdrive.setHeading(shootingHeading);

        intake.setSpeed(IntakeConstants.INTAKINGSPEED);

        if ( Math.abs(currentRobotHeading.getRadians()- shootingHeading.getRadians()) <= Drivebase.HEADING_TOLERANCE && shootButton){
          currentState = State.SHOOTING;
        }
        if (!aimButton && ! shootButton){
          currentState = State.IDLE;
        }
      break;

      case SHOOTING:

        if(shooter.shooterAtSpeed()){
          shooter.setIndexerSpeed(ShooterConstants.INDEXINGSPEED);
        }

        if(!shootButton){
          currentState = State.IDLE;
        }
      break;




    }

    


  }


  private double findTimeofFlightofProjectile(SwerveBase swerve, Translation3d target){
    ChassisSpeeds targetRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotVelocity(), swerve.getPose().getRotation().plus(findStationaryshootingHeading(swerve, target)));

    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

    double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getX() - target.getX());

    double A = - Math.pow(Constants.GRAVITY/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()/2,2);
    double B = 0;
    double C = Math.pow(targetRelativeVelocity.vxMetersPerSecond,2) + Math.pow(targetRelativeVelocity.vyMetersPerSecond,2) - Math.pow(1/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan(), 2) * Constants.GRAVITY * (target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ());
    double D = -2 * targetRelativeVelocity.vxMetersPerSecond * targetDistance;
    double E = Math.pow(targetDistance,2) - Math.pow((target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ()/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()),2);
    
    SmartDashboard.putNumber("TargetRelativeVX",targetRelativeVelocity.vxMetersPerSecond);
    
    SmartDashboard.putNumber("TargetRelativeVY",targetRelativeVelocity.vyMetersPerSecond);

    swerve.field.getObject("shooterPose").setPose(shooterPose);

    double guess = 2;
    for(int n = 0; n < ShooterConstants.N_NEWTONLINERIZATIONS; n++){
      
      guess = -(A*Math.pow(guess, 4) + B*Math.pow(guess, 3) +  C*Math.pow(guess, 2) + D*guess + E)/(4* A* Math.pow(guess,3) + 3* B* Math.pow(guess,2) + 2 * C* Math.pow(guess,1) + D) + guess;

    }

    SmartDashboard.putNumber("timeOfFlight", guess);
    SmartDashboard.putNumber("targetDistance", targetDistance);
    return guess;

  }
  private double findStationaryShootingVelocity(SwerveBase swerve, Translation3d target){

    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

    double targetDistance = Math.hypot(shooterPose.getX() - shooterPose.getX(), shooterPose.getX() - target.getX());

    double targetZ = target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ();

    return Math.sqrt( (- Constants.GRAVITY * Math.pow(targetDistance,2)) / (2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(), 2) * targetZ - targetDistance * ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()));

  } 

  private Rotation2d findStationaryshootingHeading(SwerveBase swerve, Translation3d target){

    Pose2d robotPose = swerve.getPose();

    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

    Rotation2d heading = Rotation2d.fromRadians(Math.atan2(shooterPose.getY() - target.getY(),shooterPose.getX() - target.getX())).rotateBy(Rotation2d.fromDegrees(180));

    return swerve.getAlliance() ==  Alliance.Blue ? heading : heading.rotateBy(Rotation2d.fromDegrees(180));

    
  }


  // Called once the command ends
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
