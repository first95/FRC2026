// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.commands.autocommands.AlignToPose;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveBase;

import static edu.wpi.first.units.Units.Rotation;

import java.sql.Driver;
import java.time.temporal.TemporalQuery;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import javax.lang.model.element.Element;

import com.pathplanner.lib.auto.CommandUtil;

import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


public final class Autos {
  /** Example static factory for an autonomous command. */
  private final AutoFactory autoFactory;
  private final SwerveBase swerve;
  private final FuelHandlerCommand fuelhandler;
  
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

 

  public Autos(SwerveBase swerve, FuelHandlerCommand fuelhandler) {
    autoFactory = new AutoFactory(
      swerve::getPose,
      swerve::resetOdometry,
      swerve::followTrajectory,
      true,
      swerve);
    
    this.swerve = swerve;
    this.fuelhandler = fuelhandler;
    
    autoFactory.bind("Shoot", new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_SHOOT_KEY, true)));
    autoFactory.bind("stopShoot",  new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_SHOOT_KEY, false)));
    autoFactory.bind("Intake", new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_INTAKE_KEY, true)));
    //autoFactory.bind("stopIntake",new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_INTAKE_KEY, false)));
  }

  
  public AutoRoutine ScorePreLoad(){
    AutoRoutine routine = autoFactory.newRoutine("ScorePreLoad");
    routine.active().onTrue(
      Commands.sequence(
      new InstantCommand(()->SmartDashboard.putBoolean(Auton.AUTO_INTAKE_KEY, true))
      ,new WaitCommand(Auton.STARTING_INTAKE_WAIT)
      ,(new InstantCommand(()->SmartDashboard.putBoolean(Auton.AUTO_INTAKE_KEY,false)))
      ,new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.USE_AUTO_SHOOT_KEY, true))
      ,(new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_SHOOT_KEY, true)))
      // ,(new WaitCommand(Auton.AUTON_STATIONARY_SCORING_WAIT_TIME))
      // ,(new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_SHOOT_KEY, false)))
      // ,(new InstantCommand(()-> SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY,false)))
      //,(new AlignToPose(new Pose2d(0,0,Rotation2d.fromDegrees(180)), swerve))
      )
    );
    return routine;
  }

  public AutoRoutine ScorePreLoadMoving(){
    AutoRoutine routine = autoFactory.newRoutine("ScorePreLoadMoving");

    AutoTrajectory Trajectory = routine.trajectory("ScorePreloadMoving");

  
    List<SwerveSample> movingTraj = new ArrayList<>();
    
    Trajectory.getRawTrajectory().samples().forEach(sample -> 
    movingTraj.add(
      new SwerveSample(
        sample.getTimestamp(), 
        sample.getPose().getX(), 
        sample.getPose().getY(),
        fuelhandler.findMovingShootingHeading(
          sample.getPose(), 
          sample.getChassisSpeeds(), 
          Alliance.Blue,
          fuelhandler.targetChooser(sample.getPose(),Alliance.Blue), 
          fuelhandler.findMovingShootingVelocity(sample.getPose(),sample.getChassisSpeeds(), Alliance.Blue, fuelhandler.targetChooser(sample.getPose(),Alliance.Blue)))
          .getRadians(), 
        sample.getChassisSpeeds().vxMetersPerSecond,
        sample.getChassisSpeeds().vyMetersPerSecond, 
        0, 
        0, 
        0, 
        0, 
        null, 
        null)));

    AutoTrajectory traj = routine.trajectory(new Trajectory<SwerveSample>("traj",movingTraj,null,Trajectory.getRawTrajectory().events()));


    routine.active().onTrue(
      Commands.sequence(
          new AlignToPose(traj.getInitialPose().get(), swerve),
          new InstantCommand(()-> SmartDashboard.putBoolean(Constants.Auton.AUTO_SHOOT_KEY, true)),
          traj.cmd()
          
        )
     
    );

    return routine;

  }


  public AutoRoutine leftSideMidFieldAuto(){
    SmartDashboard.putString("currentModularAuto", "SS0,SS0,SS0x,");
    return ModularAuto();
  }

  public AutoRoutine leftSideMidSweepingAuto(){
     SmartDashboard.putString("currentModularAuto", "SS0,SS0,LM0,SS0,");
    return ModularAuto();
  }

  public AutoRoutine rightSideMidFieldAuto(){
    SmartDashboard.putString("currentModularAuto", "SS1,SS1,SS1x,");
    return ModularAuto();
  }

  public AutoRoutine rightSideMidSweepingAuto(){
     SmartDashboard.putString("currentModularAuto", "SS1,LM3,SS1,LM4,SS1,");
    return ModularAuto();
  }

  public AutoRoutine ModularAuto(){
    AutoRoutine routine = autoFactory.newRoutine("ModularAuto");
    String[] posTargets = getPosTargets();
    
    Pose2d[] fullTrajectory = {};    
    if (posTargets != null && posTargets.length >= 2){
      AutoTrajectory[] trajectories = new AutoTrajectory[posTargets.length - 1];


      //load trajectorys based on posTargets
      for(int n = 0; n < trajectories.length; n++){
        
        trajectories[n] = routine.trajectory(posTargets[n].substring(1) + "x" + posTargets[n+1].substring(1));
       
        
        
        Pose2d[] trajectoryPose2dList = trajectories[n].getRawTrajectory().getPoses();
        fullTrajectory = Arrays.copyOf(fullTrajectory, fullTrajectory.length + trajectoryPose2dList.length );
        System.arraycopy(trajectoryPose2dList, 0, fullTrajectory, fullTrajectory.length - trajectoryPose2dList.length , trajectoryPose2dList.length);
      } 
      
     
      //When the routine starts run the first trajectory
      routine.active().onTrue(
        Commands.sequence(
          //new AlignToPose(trajectories[0].getInitialPose().get(), swerve),
          new InstantCommand(()->SmartDashboard.putBoolean(Auton.AUTO_INTAKE_KEY, true))
          ,new WaitCommand(Auton.STARTING_INTAKE_WAIT)
          //new WaitCommand(Auton.STARTING_INTAKE_WAIT),
          //,(new InstantCommand(()->SmartDashboard.putBoolean(Auton.AUTO_INTAKE_KEY,false)))
          ,new InstantCommand(()->SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY, true))
          .andThen(posTargets[0].charAt(0) == 'S' ? (stationaryShotRoutine(trajectories[0].getInitialPose().get(),Auton.AUTON_PRELOADSCORE_WAIT_TIME)): (new AlignToPose(trajectories[0].getInitialPose().get(), swerve)))
          ,new InstantCommand(()->SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY, false)),
          trajectories[0].cmd()
        )
      );
      //go through all trajectorys and run them one after another
      for(int n = 0; n < trajectories.length-1; n++){
        
        if(posTargets[n+1].charAt(0) == 'S'){

          trajectories[n].done().onTrue(
            stationaryShotRoutine(trajectories[n+1].getInitialPose().get(),Auton.AUTON_STATIONARY_SCORING_WAIT_TIME)
            .andThen(trajectories[n+1].cmd())
          );

        }
        else if (posTargets[n+1].charAt(0) == 'M'){
          if(!trajectories[n+1].getRawTrajectory().getEvents("Shoot").isEmpty()){
            AutoTrajectory trajo = routine.trajectory(new Trajectory<SwerveSample>("launchOnTheFlyTraj" + posTargets[n+1] + "x" + posTargets[n+2],getLaunchOnFlyTraj(trajectories[n+1],trajectories[n+1].getRawTrajectory().getEvents("Shoot").get(0).timestamp,trajectories[n+1].getRawTrajectory().getEvents("stopShoot").get(0).timestamp),null,trajectories[n+1].getRawTrajectory().events()));
            trajectories[n+1] = trajo;
          }
          
          
          trajectories[n].done().onTrue(
            new InstantCommand(()-> SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY,true))
            .andThen(trajectories[n+1].cmd()
            .andThen(new InstantCommand(()-> SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY,false)))));
        }
        else{
          trajectories[n].done().onTrue(new InstantCommand(()-> SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY,false)).andThen(trajectories[n+1].cmd()));
        }

        if(posTargets[posTargets.length-1].charAt(0) == 'C'){
          
          trajectories[trajectories.length-1].done().onTrue(
            new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_CLIMB_KEY, true))
            .andThen(new AlignToPose(new Pose2d(Auton.POSE_MAP.get(swerve.getAlliance()).get("climb").toTranslation2d(), Rotation2d.fromDegrees(Auton.POSE_MAP.get(swerve.getAlliance()).get("climb").getZ()).rotateBy(swerve.getAlliance() == Alliance.Blue ? new Rotation2d(1,0): new Rotation2d(-1,0))), swerve))
            .andThen(new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_CLIMB_KEY, true)))
          );

        }
        else if(posTargets[posTargets.length-1].charAt(0) == 'S'){
          
          trajectories[trajectories.length-1].done().onTrue(
            stationaryShotRoutine(trajectories[trajectories.length-1].getFinalPose().get())
          );

        }
        
        //trajectories[n].done().onTrue(trajectories[n+1].cmd());
      }
      
      swerve.field.getObject("autoTrajectory").setPoses(fullTrajectory);
      swerve.field.getObject("target").setPose(trajectories[0].getInitialPose().get());
    }
    
    return routine;  
  }
  private Command stationaryShotRoutine(Pose2d pose){
    return stationaryShotRoutine(pose, 0, false);
  }
  private Command stationaryShotRoutine(Pose2d pose, double waitTime){
    return stationaryShotRoutine(pose, waitTime, true);
  }
  private Command stationaryShotRoutine(Pose2d pose, double waitTime, boolean end){
    return new InstantCommand(()-> SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY,true))
            .andThen(new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_SHOOT_KEY, true)))
            .andThen(new AlignToPose(new Pose2d(pose.getX(),pose.getY(),fuelhandler.findStationaryShootingHeading(pose, swerve.getAlliance(), fuelhandler.targetChooser(pose,swerve.getAlliance())).rotateBy(Rotation2d.fromDegrees(swerve.getAlliance() == Alliance.Blue ? 0: 180))), swerve))
            .andThen(!end?new InstantCommand():((new WaitCommand(waitTime))
            .andThen(new InstantCommand(() -> SmartDashboard.putBoolean(Auton.AUTO_SHOOT_KEY, false)))
            .andThen(new InstantCommand(()-> SmartDashboard.putBoolean(Auton.USE_AUTO_SHOOT_KEY,false)))));
  }
  private String[] getPosTargets(){
    String currentModularAuto = SmartDashboard.getString("currentModularAuto", "");
    String[] posTargets = {};
    String posTarget = "";
    if(currentModularAuto.length() > 3){
      for(int character = 0; character < currentModularAuto.length(); character ++){
      
        if(currentModularAuto.charAt(character) == ','){
          int posTargetsLength = posTargets.length;
          posTargets = Arrays.copyOf(posTargets,posTargetsLength + 1);
          posTargets[posTargetsLength] = posTarget;
          posTarget = "";
        }
        else{
          posTarget += currentModularAuto.charAt(character);
        }
      }
      SmartDashboard.putStringArray("posTargets", posTargets);
      return posTargets;
      
    }
    else{
      return new String[0];
    }
    
    
  }
  private List<SwerveSample> getLaunchOnFlyTraj(AutoTrajectory ogTraj,double shootStart,double shootEnd){
    List<SwerveSample> movingTraj = new ArrayList<>();
    
    ogTraj.getRawTrajectory().samples().forEach(sample -> 
      movingTraj.add(
        sample.getTimestamp()<shootEnd && sample.getTimestamp()>shootStart?
        new SwerveSample(
        sample.getTimestamp(), 
        sample.getPose().getX(), 
        sample.getPose().getY(),
        fuelhandler.findMovingShootingHeading(
        sample.getPose(), 
        sample.getChassisSpeeds(), 
        Alliance.Blue,
        fuelhandler.targetChooser(sample.getPose(),Alliance.Blue), 
        fuelhandler.findMovingShootingVelocity(sample.getPose(),sample.getChassisSpeeds(),Alliance.Blue, fuelhandler.targetChooser(sample.getPose(),Alliance.Blue)))
        .getRadians(), 
        sample.getChassisSpeeds().vxMetersPerSecond,
        sample.getChassisSpeeds().vyMetersPerSecond, 
        0, 
        0, 
        0, 
        0, 
        null, 
        null)
        :
        new SwerveSample(
        sample.getTimestamp(), 
        sample.getPose().getX(), 
        sample.getPose().getY(),
        sample.getPose().getRotation().getRadians(), 
        sample.getChassisSpeeds().vxMetersPerSecond,
        sample.getChassisSpeeds().vyMetersPerSecond, 
        sample.getChassisSpeeds().omegaRadiansPerSecond, 
        0, 
        0, 
        0, 
        null, 
        null)));
    SwerveSample finalSample = movingTraj.get(movingTraj.size() - 1);
  
    movingTraj.set(movingTraj.size()-1,new SwerveSample(finalSample.getTimestamp(), finalSample.getPose().getX(), finalSample.getPose().getY(), finalSample.getPose().getRotation().getRadians(), 0, 0, 0, 0, 0, 0, null, null));
    return movingTraj;

  }
}
