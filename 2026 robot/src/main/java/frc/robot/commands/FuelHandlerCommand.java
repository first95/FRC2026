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

import static edu.wpi.first.units.Units.Rotation;

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

  private Translation3d target;

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
    //addRequirements(intake);
  }


;
  public void initialize() {
    currentState = State.IDLE;
  }


  public void execute() {

    SmartDashboard.putNumber("shooterExitVelocity", findMovingShootingVelocity(swerve, Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub"))) ;
    SmartDashboard.putNumber("stationaryExitVelocity",findStationaryShootingVelocity(swerve,Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub")));
    intakeButton = intakeButtonSupplier.getAsBoolean();
    aimButton = aimButtonSupplier.getAsBoolean();
    shootButton = shootButtonSupplier.getAsBoolean();

    currentRobotHeading = swerve.getPose().getRotation();

    target = targetChooser(swerve);
    shootingVelocity = findMovingShootingVelocity(swerve, target);
    shootingHeading = findMovingShootingHeading(swerve, target, shootingVelocity);
    //shootingHeading = findStationaryshootingHeading(swerve,  Constants.Auton.BLUEHUB);


    switch(currentState){

      case IDLE: 
        shooter.setIndexerSpeed(0);
        shooter.setShooterExitVelocity(0);
        absdrive.setCenterOfRotation(Constants.Drivebase.CENTEROFROTATION);
        absdrive.setLocustDriving(false);

        if (intakeButton){
          currentState = State.INTAKING;
        }

        if( shootButton || aimButton){
          currentState = State.AIMING;
        }
      break;

      case INTAKING:
        absdrive.setLocustDriving(true);
        //intake.setSpeed(IntakeConstants.INTAKINGSPEED);
        shooter.setTopRollerRaw(ShooterConstants.TOPROLLER_JUGGLING_SPEED);
        shooter.setBottomRollerRaw(ShooterConstants.BOTTOMROLLER_JUGGLING_SPEED);

        if(!intakeButton){
          currentState = State.IDLE;
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

        //intake.setSpeed(IntakeConstants.INTAKINGSPEED);
        //intake.setAgitatorSpeed(IntakeConstants.AGITATINGSPEED);

        if ( shooter.shooterAtSpeed() && Math.abs(currentRobotHeading.getRadians()- shootingHeading.getRadians()) <= Drivebase.HEADING_TOLERANCE && shootButton){
          currentState = State.SHOOTING;
        }
        if (!aimButton && ! shootButton){
          currentState = State.IDLE;
        }
      break;

      case SHOOTING:
        shooter.setShooterExitVelocity(shootingVelocity);
        absdrive.setCenterOfRotation(ShooterConstants.SHOOTERLOCATION.toTranslation2d());
        absdrive.setHeading(shootingHeading);

        shooter.setIndexerPID(ShooterConstants.INDEXINGSPEED);

        if(!shooter.shooterAtSpeed() || Math.abs(currentRobotHeading.getRadians() - shootingHeading.getRadians()) > Drivebase.HEADING_TOLERANCE){
          currentState = State.AIMING;
        }

        if(!shootButton){
          currentState = State.IDLE;
        }
      break;




    }

    


  }


  private double findTimeofFlightofProjectile(SwerveBase swerve, Translation3d target){
    ChassisSpeeds swerveVelocity = swerve.getFieldVelocity();
    
    Rotation2d robotToTargetHeading = findStationaryshootingHeading(swerve, target);
    double trvx = -1*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
    double trvy = robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond;

    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

    double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getY() - target.getY());

    double A = - Math.pow(Constants.GRAVITY/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()/2,2);
    double B = 0;
    double C = Math.pow(trvx,2) + Math.pow(trvy,2) - Math.pow(1/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan(), 2) * Constants.GRAVITY * (target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ());
    double D = -2 * trvx * targetDistance;
    double E = Math.pow(targetDistance,2) - Math.pow((target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ()/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()),2);
    
    SmartDashboard.putNumber("TargetRelativeVX",trvx);
    
    SmartDashboard.putNumber("TargetRelativeVY",trvy);

    swerve.field.getObject("shooterPose").setPose(shooterPose);

    double guess = 2;
    for(int n = 0; n < ShooterConstants.N_NEWTONLINERIZATIONS; n++){
      
      guess = -(A*Math.pow(guess, 4) + B*Math.pow(guess, 3) +  C*Math.pow(guess, 2) + D*guess + E)/(4* A* Math.pow(guess,3) + 3* B* Math.pow(guess,2) + 2 * C * Math.pow(guess,1) + D) + guess;

    }

    SmartDashboard.putNumber("timeOfFlight", guess);
    SmartDashboard.putNumber("targetDistance", targetDistance);
    return guess;

  }
  private double findMovingShootingVelocity(SwerveBase swerve, Translation3d target){

    double timeOfFlight = findTimeofFlightofProjectile(swerve,target);

    ChassisSpeeds swerveVelocity = swerve.getFieldVelocity();

    Rotation2d robotToTargetHeading = findStationaryshootingHeading(swerve, target);
    double trvx = -1*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
    double trvy = robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond;


    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);
    
    double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getY() - target.getY());
    double fakeTargetDistance = Math.hypot(trvy*timeOfFlight, targetDistance - trvx*timeOfFlight);

    double targetZ = target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ();

    SmartDashboard.putNumber("fakeTargetDistance", fakeTargetDistance);

    double a = Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(),2) * targetZ - targetDistance*ShooterConstants.SHOOTER_EXIT_ANGLE.getCos()*ShooterConstants.SHOOTER_EXIT_ANGLE.getSin();
    double b = 2*targetZ*ShooterConstants.SHOOTER_EXIT_ANGLE.getCos() * trvx - targetDistance * ShooterConstants.SHOOTER_EXIT_ANGLE.getSin()* trvx;
    double c = Math.pow(trvx,2) * targetZ + Math.pow(targetDistance,2)*Constants.GRAVITY/2;

    return Math.sqrt( (- Constants.GRAVITY * Math.pow(fakeTargetDistance,2)) / 
    (2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(), 2) * (targetZ - fakeTargetDistance * ShooterConstants.SHOOTER_EXIT_ANGLE.getTan())));
    //return ((-b - Math.sqrt(Math.pow(b,2)-4*a*c))/2*a);
    // return Math.sqrt((-1 * Constants.GRAVITY* Math.pow(fakeTargetDistance, 2))/
    // (2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(),2) *  targetZ - 2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(),2) *fakeTargetDistance*ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()));

  } 

  private Rotation2d findMovingShootingHeading(SwerveBase swerve, Translation3d target, double shootingVelocity){
    ChassisSpeeds swerveVelocity = swerve.getFieldVelocity();

    Rotation2d robotToTargetHeading = findStationaryshootingHeading(swerve, target);
    double trvx = robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond;
    double trvy = robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond;

    Rotation2d heading = findStationaryshootingHeading(swerve, target).minus(Rotation2d.fromRadians(Math.PI/2 - Math.acos(-1 * trvy/ShooterConstants.SHOOTER_EXIT_ANGLE.getCos()/shootingVelocity)));

    return swerve.getAlliance() ==  Alliance.Blue ? heading : heading.rotateBy(Rotation2d.fromDegrees(180));
  }

  private double findStationaryShootingVelocity(SwerveBase swerve, Translation3d target){

    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

    double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getY() - target.getY());

    double targetZ = target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ();

    return Math.sqrt( (- Constants.GRAVITY * Math.pow(targetDistance,2)) / 
    (2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(), 2) * (targetZ - targetDistance * ShooterConstants.SHOOTER_EXIT_ANGLE.getTan())));

  } 

  private Rotation2d findStationaryshootingHeading(SwerveBase swerve, Translation3d target){

    Pose2d robotPose = swerve.getPose();

    Pose2d shooterPose = robotPose.plus(ShooterConstants.SHOOTERTRANSFORM);

    Rotation2d heading = Rotation2d.fromRadians(Math.atan2(shooterPose.getY() - target.getY(),shooterPose.getX() - target.getX())).rotateBy(Rotation2d.fromDegrees(180));

    return heading;
    //swerve.getAlliance() ==  Alliance.Red ? heading : heading.rotateBy(Rotation2d.fromDegrees(180));
  }
  private Translation3d targetChooser(SwerveBase swerve){
    Translation3d target = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub");;
    double swerveX = swerve.getPose().getX();
    double swerveY = swerve.getPose().getY();
    if(swerve.getAlliance() == Alliance.Blue){
      if( swerveX > Constants.Auton.POSE_MAP.get(Alliance.Blue).get("passingTarget1").getX()){
        if(swerveY > Constants.FIELD_WIDTH/2){
           target = Constants.Auton.POSE_MAP.get(Alliance.Blue).get("passingTarget1");
        }
        else{
          target = Constants.Auton.POSE_MAP.get(Alliance.Blue).get("passingTarget2");
        }
      }
      else{
        target = Constants.Auton.POSE_MAP.get(Alliance.Blue).get("Hub");
      }
    }
    else if(swerve.getAlliance() == Alliance.Red){
      if(swerveX < Constants.Auton.POSE_MAP.get(Alliance.Red).get("passingTarget1").getX()){
        if(swerveY < Constants.FIELD_WIDTH/2){
           target = Constants.Auton.POSE_MAP.get(Alliance.Red).get("passingTarget1");
        }
        else{
          target = Constants.Auton.POSE_MAP.get(Alliance.Red).get("passingTarget2");
        }

      }
      else{
         target = Constants.Auton.POSE_MAP.get(Alliance.Red).get("Hub");
      }
    }


    return target;

  }
  


  // Called once the command ends
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
