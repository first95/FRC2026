// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;

import static edu.wpi.first.units.Units.Rotation;

import java.sql.Driver;
import java.util.function.BooleanSupplier;

import javax.sound.sampled.TargetDataLine;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
public class FuelHandlerCommand extends Command {
  
  private final Intake intake;
  private final Shooter shooter;
  private final AbsoluteDrive absdrive;
  private final SwerveBase swerve;

  private BooleanSupplier 
    intakeButtonSupplier, 
    shootButtonSupplier,
    aimButtonSupplier,
    autoHubOverideSupplier;
  
  private boolean
    intakeButton,
    shootButton,
    aimButton,
    autoHubOverideButton,
    autoHubOveride = false,
    hubActive,
    inRange;
  
  private Rotation2d 
  currentRobotHeading, 
  shootingHeading;

  private double 
  shootingVelocity,
  shootingRPMOffset,
  timeOfFlightToHub;

  private Translation3d target;

  private enum State{
    IDLE, INTAKING, AIMING, SHOOTING
  }

  private State currentState;

  public FuelHandlerCommand(
    BooleanSupplier intakeButtonSupplier,
    BooleanSupplier shootButtonSupplier,
    BooleanSupplier aimButtonSupplier,
    BooleanSupplier autoHubOverideSupplier,
    Shooter shooter,
    Intake intake,
    AbsoluteDrive absdrive,
    SwerveBase swerve) {

    this.intakeButtonSupplier = intakeButtonSupplier;
    this.shootButtonSupplier = shootButtonSupplier;
    this.aimButtonSupplier = aimButtonSupplier;
    this.autoHubOverideSupplier = autoHubOverideSupplier;

    this.shooter = shooter;
    this.intake = intake;
    this.absdrive = absdrive;
    this.swerve = swerve;
    
    addRequirements(shooter);
    addRequirements(intake);
  }


;
  public void initialize() {
    currentState = State.IDLE;
  }


  public void execute() {

    
    SmartDashboard.putNumber("shooterExitVelocity", findMovingShootingVelocity(swerve, Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub"))) ;
    SmartDashboard.putNumber("stationaryExitVelocity",findStationaryShootingVelocity(swerve,Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub")));
    
    autoHubOverideButton = autoHubOverideSupplier.getAsBoolean();

    if(autoHubOverideButton){
      autoHubOveride = true;
      inRange = true;
    }
    

    if (SmartDashboard.getBoolean("autoEnabled",false)){
      intakeButton = SmartDashboard.getBoolean(Constants.Auton.AUTO_INTAKE_KEY, false);
      shootButton = SmartDashboard.getBoolean(Constants.Auton.AUTO_SHOOT_KEY, false)&&SmartDashboard.getBoolean(Constants.Auton.USE_AUTO_SHOOT_KEY, false);
    }
    else{
      intakeButton = intakeButtonSupplier.getAsBoolean();
      aimButton = aimButtonSupplier.getAsBoolean();
      shootButton = shootButtonSupplier.getAsBoolean();
    }
    
    
    currentRobotHeading = swerve.getPose().getRotation();

    target = targetChooser(swerve);
    shootingVelocity = findMovingShootingVelocity(swerve, target);
    shootingHeading = findMovingShootingHeading(swerve, target, shootingVelocity);
    timeOfFlightToHub = findTimeofFlightofProjectile(swerve, Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub"));

    hubActive = autoHubOveride ? true : canLaunch(swerve, timeOfFlightToHub);
    displayTimeToNextShift(timeOfFlightToHub);
    inRange = findFakeTargetDistance(swerve, timeOfFlightToHub, Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub")) > ShooterConstants.MINRANGE;

    //shootingHeading = findStationaryshootingHeading(swerve,target);

    SmartDashboard.putBoolean("shouldLaunch", canLaunch(swerve, timeOfFlightToHub));

    SmartDashboard.putNumber("shootingHeading", shootingHeading.getRadians());
    swerve.field.getObject("target").setPose(new Pose2d(target.toTranslation2d(),new Rotation2d()));
    //shootingHeading = findStationaryshootingHeading(swerve,  Constants.Auton.BLUEHUB);




    switch(currentState){

      case IDLE: 

        shooter.setIndexerSpeed(0);
        shooter.setShooterExitVelocity(0);

        absdrive.setCenterOfRotation(Constants.Drivebase.CENTEROFROTATION);
        absdrive.setLocustDriving(false);

        intake.setSpeed(IntakeConstants.INTAKEIDLESPEED);
        intake.setAgitator1Speed(IntakeConstants.AGITATOR1IDLESPEED);
        intake.setAgitator2Speed(IntakeConstants.AGITATOR2IDLESPEED);


        if (intakeButton){
          currentState = State.INTAKING;
        }

        if( shootButton || aimButton){
          currentState = State.AIMING;
        }
      break;

      case INTAKING:
        shooter.setIndexerPID(ShooterConstants.INDEXINGSPEED);
        shooter.setShooterSpeeds(ShooterConstants.TOPROLLER_JUGGLING_SPEED, ShooterConstants.BOTTOMROLLER_JUGGLING_SPEED);

        absdrive.setLocustDriving(false);


        intake.setSpeed(IntakeConstants.INTAKINGSPEED);
        intake.setAgitator1Speed(IntakeConstants.AGITATOR1INTAKINGSPEED);
        intake.setAgitator2Speed(IntakeConstants.AGITATOR2INTAKINGSPEED);

        

        if(!intakeButton){
          currentState = State.IDLE;
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

        intake.setSpeed(IntakeConstants.INTAKEAIMINGSPEED);
        intake.setAgitator1Speed(IntakeConstants.AGITATOR1AIMINGSPEED);
        intake.setAgitator2Speed(IntakeConstants.AGITATOR2AIMINGSPEED);
        

        

        if (shooter.shooterAtSpeed() && Math.abs(currentRobotHeading.minus(shootingHeading).getRadians()) <= Drivebase.HEADING_TOLERANCE && shootButton){
          if(target == Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub")){
            if (hubActive && inRange){
              currentState = State.SHOOTING;
            } 
          }
          else{
            currentState = State.SHOOTING;
          }
            
        }
        if (!aimButton && ! shootButton){
          currentState = State.IDLE;
        }
      break;

      case SHOOTING:
        shooter.setShooterExitVelocity(shootingVelocity, shootingRPMOffset);
        shooter.setIndexerPID(ShooterConstants.INDEXINGSPEED);

        absdrive.setCenterOfRotation(ShooterConstants.SHOOTERLOCATION.toTranslation2d());
        //absdrive.setHeading(shootingHeading);

        intake.setSpeed(IntakeConstants.INTAKINGSPEED);
        intake.setAgitator1Speed(IntakeConstants.AGITATOR1SHOOTINGSPEED);
        intake.setAgitator2Speed(IntakeConstants.AGITATOR2SHOOTINGSPEED);
        


        if(!shooter.shooterAtSpeed() || Math.abs(currentRobotHeading.minus(shootingHeading).getRadians()) > Drivebase.HEADING_TOLERANCE){
          currentState = State.AIMING;
        }
        if(target == Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub")){
          if (!hubActive || !inRange){
            currentState = State.AIMING;
          } 
        }

        

        if(!shootButton){
          currentState = State.IDLE;
        }
      break;




    }

    


  }


  // public double findTimeofFlightofProjectile(SwerveBase swerve, Translation3d target){
  //   ChassisSpeeds swerveVelocity = swerve.getFieldVelocity();
    
  //   Rotation2d robotToTargetHeading = findStationaryshootingHeading(swerve, target);
  //   double trvx = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
  //   double trvy = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond);

  //   Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

  //   double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getY() - target.getY());

  //   double A = -1 * Math.pow(Constants.GRAVITY/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()/2,2);
  //   double B = 0;
  //   double C = Math.pow(trvx,2) + Math.pow(trvy,2) - (Math.pow(1/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan(), 2) * Constants.GRAVITY * (target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ()));
  //   double D = -2 * trvx * targetDistance;
  //   double E = Math.pow(targetDistance,2) - Math.pow((target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ())/ShooterConstants.SHOOTER_EXIT_ANGLE.getTan(),2);
    
  //   SmartDashboard.putNumber("TargetRelativeVX",trvx);
    
  //   SmartDashboard.putNumber("TargetRelativeVY",trvy);

  //   swerve.field.getObject("shooterPose").setPose(shooterPose);

  //   double guess = 2;
  //   for(int n = 0; n < ShooterConstants.N_NEWTONLINERIZATIONS; n++){
      
  //     guess = -1 * (A*Math.pow(guess, 4) + B*Math.pow(guess, 3) +  C*Math.pow(guess, 2) + D*guess + E)/(4* A* Math.pow(guess,3) + 3* B* Math.pow(guess,2) + 2 * C * Math.pow(guess,1) + D) + guess;

  //   }

  //   SmartDashboard.putNumber("timeOfFlight", guess);
  //   SmartDashboard.putNumber("targetDistance", targetDistance);
  //   return guess;

  // }
  public double findTimeofFlightofProjectile(SwerveBase swerve, Translation3d target){
    return findTimeofFlightofProjectile(swerve.getPose(),swerve.getFieldVelocity(), swerve.getAlliance(), target);
  }
  public double findTimeofFlightofProjectile(Pose2d pose, ChassisSpeeds fieldVelocity,Alliance alliance, Translation3d target){
    ChassisSpeeds swerveVelocity = fieldVelocity; 
    
    Rotation2d robotToTargetHeading = findStationaryShootingHeading(pose,alliance, target);
    double trvx = (alliance == Alliance.Blue? 1: -1)*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
    double trvy = (alliance == Alliance.Blue? 1: -1)*(robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond);

    Pose2d shooterPose = pose.plus(ShooterConstants.SHOOTERTRANSFORM);

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

  public double findFakeTargetDistance(SwerveBase swerve, double timeOfFlight, Translation3d target){

    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

    Rotation2d robotToTargetHeading = findStationaryShootingHeading(swerve, target);

    ChassisSpeeds swerveVelocity = swerve.getFieldVelocity();

    double trvx = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
    double trvy = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond);
    
    double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getY() - target.getY());

    return Math.hypot(trvy*timeOfFlight, targetDistance - trvx*timeOfFlight);
  }

  // public double findMovingShootingVelocity(SwerveBase swerve, Translation3d target){

  //   double timeOfFlight = findTimeofFlightofProjectile(swerve,target);

  //   ChassisSpeeds swerveVelocity = swerve.getFieldVelocity();

  //   Rotation2d robotToTargetHeading = findStationaryshootingHeading(swerve, target);
  //   double trvx = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
  //   double trvy = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond);


  //   Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);
    
  //   double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getY() - target.getY());
  //   double fakeTargetDistance = Math.hypot(trvy*timeOfFlight, targetDistance - trvx*timeOfFlight);

  //   double targetZ = target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ();

  //   SmartDashboard.putNumber("fakeTargetDistance", fakeTargetDistance);

  //   double a = Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(),2) * targetZ - targetDistance*ShooterConstants.SHOOTER_EXIT_ANGLE.getCos()*ShooterConstants.SHOOTER_EXIT_ANGLE.getSin();
  //   double b = 2*targetZ*ShooterConstants.SHOOTER_EXIT_ANGLE.getCos() * trvx - targetDistance * ShooterConstants.SHOOTER_EXIT_ANGLE.getSin()* trvx;
  //   double c = Math.pow(trvx,2) * targetZ + Math.pow(targetDistance,2)*Constants.GRAVITY/2;

  //   return Math.sqrt( (- Constants.GRAVITY * Math.pow(fakeTargetDistance,2)) / 
  //   (2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(), 2) * (targetZ - fakeTargetDistance * ShooterConstants.SHOOTER_EXIT_ANGLE.getTan())));
  //   //return ((-b - Math.sqrt(Math.pow(b,2)-4*a*c))/2*a);
  //   // return Math.sqrt((-1 * Constants.GRAVITY* Math.pow(fakeTargetDistance, 2))/
  //   // (2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(),2) *  targetZ - 2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(),2) *fakeTargetDistance*ShooterConstants.SHOOTER_EXIT_ANGLE.getTan()));

  // } 
  public double findMovingShootingVelocity(SwerveBase swerve, Translation3d target){
    return findMovingShootingVelocity(swerve.getPose(),swerve.getFieldVelocity(),swerve.getAlliance(), target);
  }

  public double findMovingShootingVelocity(Pose2d pose, ChassisSpeeds fieldVelocity, Alliance alliance, Translation3d target){

    double timeOfFlight = findTimeofFlightofProjectile(pose, fieldVelocity, alliance, target);

    ChassisSpeeds swerveVelocity = fieldVelocity;

    Rotation2d robotToTargetHeading = findStationaryShootingHeading(pose,alliance, target);
    double trvx = (alliance == Alliance.Blue? 1: -1)*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
    double trvy = (alliance == Alliance.Blue? 1: -1)*(robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond);


    Pose2d shooterPose = pose.plus(ShooterConstants.SHOOTERTRANSFORM);
    
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

  // public Rotation2d findMovingShootingHeading(SwerveBase swerve, Translation3d target, double shootingVelocity){
  //   ChassisSpeeds swerveVelocity = swerve.getFieldVelocity();

  //   Rotation2d robotToTargetHeading = findStationaryshootingHeading(swerve, target);
  //   double trvx = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
  //   double trvy = (swerve.getAlliance() == Alliance.Blue? 1: -1)*(robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond);

  //   Rotation2d heading = findStationaryshootingHeading(swerve, target).minus(Rotation2d.fromRadians(Math.PI/2 - Math.acos(-1 * trvy/ShooterConstants.SHOOTER_EXIT_ANGLE.getCos()/shootingVelocity)));
  //   Rotation2d headingRed = findStationaryshootingHeading(swerve, target).minus(Rotation2d.fromRadians(Math.PI/2 - Math.acos(-1 * trvy/ShooterConstants.SHOOTER_EXIT_ANGLE.getCos()/shootingVelocity)));
  //   //return swerve.getAlliance() ==  Alliance.Blue ? heading : heading.rotateBy(Rotation2d.fromDegrees(180));
  //   return heading;
  // }

  public Rotation2d findMovingShootingHeading(SwerveBase swerve, Translation3d target, double shootingVelocity){
    return findMovingShootingHeading(swerve.getPose(),swerve.getFieldVelocity(),swerve.getAlliance(), target, shootingVelocity);
  }
  public Rotation2d findMovingShootingHeading(Pose2d pose, ChassisSpeeds fieldVelocity,Alliance alliance, Translation3d target, double shootingVelocity){
    ChassisSpeeds swerveVelocity = fieldVelocity;

    Rotation2d robotToTargetHeading = findStationaryShootingHeading(pose,alliance, target);
    double trvx = (alliance == Alliance.Blue? 1: -1)*(robotToTargetHeading.getCos()*swerveVelocity.vxMetersPerSecond + robotToTargetHeading.getSin()*swerveVelocity.vyMetersPerSecond);
    double trvy = (alliance == Alliance.Blue? 1: -1)*(robotToTargetHeading.getSin()*swerveVelocity.vxMetersPerSecond - robotToTargetHeading.getCos()*swerveVelocity.vyMetersPerSecond);

    Rotation2d heading = robotToTargetHeading.minus(Rotation2d.fromRadians(Math.PI/2 - Math.acos(-1 * trvy/ShooterConstants.SHOOTER_EXIT_ANGLE.getCos()/shootingVelocity)));

    //return swerve.getAlliance() ==  Alliance.Blue ? heading : heading.rotateBy(Rotation2d.fromDegrees(180));
    return heading;
  }

  public double findStationaryShootingVelocity(SwerveBase swerve, Translation3d target){

    Pose2d shooterPose = swerve.getPose().plus(ShooterConstants.SHOOTERTRANSFORM);

    double targetDistance = Math.hypot(shooterPose.getX() - target.getX(), shooterPose.getY() - target.getY());

    double targetZ = target.getZ() - ShooterConstants.SHOOTERLOCATION.getZ();

    return Math.sqrt( (- Constants.GRAVITY * Math.pow(targetDistance,2)) / 
    (2 * Math.pow(ShooterConstants.SHOOTER_EXIT_ANGLE.getCos(), 2) * (targetZ - targetDistance * ShooterConstants.SHOOTER_EXIT_ANGLE.getTan())));

  } 

  // public Rotation2d findStationaryshootingHeading(SwerveBase swerve, Translation3d target){

  //   Pose2d robotPose = swerve.getPose();

  //   Pose2d shooterPose = robotPose.plus(ShooterConstants.SHOOTERTRANSFORM);

  //   Rotation2d heading = Rotation2d.fromRadians(Math.atan2(shooterPose.getY() - target.getY(),shooterPose.getX() - target.getX())).rotateBy(Rotation2d.fromDegrees(180));

  //   //return heading;
  //   return swerve.getAlliance() ==  Alliance.Blue ? heading : heading.rotateBy(Rotation2d.fromDegrees(180));
  // }
  public Rotation2d findStationaryShootingHeading(SwerveBase swerve, Translation3d target){
    return findStationaryShootingHeading(swerve.getPose(), swerve.getAlliance(), target);
  }
  public Rotation2d findStationaryShootingHeading(Pose2d pose,Alliance alliance, Translation3d target){

    Pose2d robotPose = pose;

    Pose2d shooterPose = robotPose.plus(ShooterConstants.SHOOTERTRANSFORM);

    Rotation2d heading = Rotation2d.fromRadians(Math.atan2(shooterPose.getY() - target.getY(),shooterPose.getX() - target.getX())).rotateBy(Rotation2d.fromDegrees(180));

    //return heading;
    return alliance ==  Alliance.Blue ? heading : heading.rotateBy(Rotation2d.fromDegrees(180));
  }

  // public Translation3d targetChooser(SwerveBase swerve){
  //   Translation3d target = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Hub");;
  //   double swerveX = swerve.getPose().getX();
  //   double swerveY = swerve.getPose().getY();
  //   if(swerve.getAlliance() == Alliance.Blue){
  //     if( swerveX > Constants.Auton.POSE_MAP.get(Alliance.Blue).get("passingTarget1").getX()){
  //       if(swerveY > Constants.FIELD_WIDTH/2){
  //          target = Constants.Auton.POSE_MAP.get(Alliance.Blue).get("passingTarget1");
  //       }
  //       else{
  //         target = Constants.Auton.POSE_MAP.get(Alliance.Blue).get("passingTarget2");
  //       }
  //     }
  //     else{
  //       target = Constants.Auton.POSE_MAP.get(Alliance.Blue).get("Hub");
  //     }
  //   }
  //   else if(swerve.getAlliance() == Alliance.Red){
  //     if(swerveX < Constants.Auton.POSE_MAP.get(Alliance.Red).get("passingTarget1").getX()){
  //       if(swerveY < Constants.FIELD_WIDTH/2){
  //          target = Constants.Auton.POSE_MAP.get(Alliance.Red).get("passingTarget1");
  //       }
  //       else{
  //         target = Constants.Auton.POSE_MAP.get(Alliance.Red).get("passingTarget2");
  //       }

  //     }
  //     else{
  //        target = Constants.Auton.POSE_MAP.get(Alliance.Red).get("Hub");
  //     }
  //   }


  //   return target;

  // }

  public Translation3d targetChooser(SwerveBase swerve){
    return targetChooser(swerve.getPose(), swerve.getAlliance());
  }

  public Translation3d targetChooser(Pose2d pose, Alliance alliance){
    Translation3d target = Constants.Auton.POSE_MAP.get(Alliance.Blue).get("Hub");;
    double swerveX = pose.getX();
    double swerveY = pose.getY();
    if(alliance == Alliance.Blue){
      if( swerveX > Constants.Auton.POSE_MAP.get(Alliance.Blue).get("Hub").getX()){
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
    else if(alliance == Alliance.Red){
      if(swerveX < Constants.Auton.POSE_MAP.get(Alliance.Red).get("Hub").getX()){
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

  private boolean canLaunch(SwerveBase swerve, double timeOfFlight){
    char alliance = swerve.getAlliance() == Alliance.Blue? 'B': 'R';

    if(SmartDashboard.getBoolean(Auton.AUTO_ENABLED_KEY, false)){
      return true;
    }

    double matchTime = DriverStation.getMatchTime();
    String autoWinner = DriverStation.getGameSpecificMessage();

    if(autoWinner.isEmpty()){
      return true;
    }
    else{
      boolean wonAuto = autoWinner.charAt(0) == alliance;
      if (matchTime > Auton.TRANSITION_ENDTIME + timeOfFlight){
        return true;
      }
      else if (matchTime > Auton.FIRSTSHIFT_ENDTIME + timeOfFlight){
        return wonAuto;
      }
      else if (matchTime > Auton.SECONDSHIFT_ENDTIME + timeOfFlight){
        return !wonAuto;
      }
      else if (matchTime > Auton.THIRDSHIFT_ENDTIME + timeOfFlight){
        return wonAuto;
      }
      else if (matchTime > Auton.FORTHSHIFT_ENDTIME + timeOfFlight){
        return !wonAuto;
      }
      else{
        return true;
      }


    }

  }
  private void displayTimeToNextShift(double timeOfFlight){

    double matchTime = DriverStation.getMatchTime();
    
    double timeToNext = 0;
    if (matchTime > Auton.TRANSITION_ENDTIME){
        timeToNext = matchTime - Auton.TRANSITION_ENDTIME;
      }
      else if (matchTime > Auton.FIRSTSHIFT_ENDTIME ){
        timeToNext = matchTime - Auton.FIRSTSHIFT_ENDTIME;
      }
      else if (matchTime > Auton.SECONDSHIFT_ENDTIME ){
        timeToNext = matchTime - Auton.SECONDSHIFT_ENDTIME;
      }
      else if (matchTime > Auton.THIRDSHIFT_ENDTIME ){
        timeToNext = matchTime - Auton.THIRDSHIFT_ENDTIME;
      }
      else if (matchTime > Auton.FORTHSHIFT_ENDTIME ){
        timeToNext = matchTime - Auton.FORTHSHIFT_ENDTIME;
      }
      else{
        timeToNext = matchTime;
      }
      SmartDashboard.putNumber("timeToNextShift", timeToNext);
      SmartDashboard.putNumber("timeToNextLaunch", timeToNext - timeOfFlight);
  }


  private void changeShootingRPMOffset(double amount){
      shootingRPMOffset += amount;
    } 
  public Command increaseShootingRPMOffsetCommand(){
    return new InstantCommand(() -> changeShootingRPMOffset(ShooterConstants.RPMOFFSET_INCREMENT));
  }
  public Command decreaseShootingRPMOffsetCommand(){
      return new InstantCommand(() -> changeShootingRPMOffset(-ShooterConstants.RPMOFFSET_INCREMENT));
  }
  


  // Called once the command ends
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
