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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  
  private Rotation2d currentRobotHeading, shootingHeading;

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
    shootingHeading = findshootingHeading(swerve, Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub"));

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
        shooter.setShooterExitVelocity(0);
        shooter.setIndexerSpeed(0);

        absdrive.setLocustDriving(false);
        absdrive.setCenterOfRotation(ShooterConstants.SHOOTERTRANSLATION2D);
        absdrive.setHeading(new Rotation2d());

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
  public Rotation2d findshootingHeading(SwerveBase swerve, Pose2d target){

    Pose2d robotPose = swerve.getPose();

    Rotation2d heading = Rotation2d.fromRadians(Math.atan2(robotPose.getY() - target.getY(),robotPose.getX() - target.getX()));

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
