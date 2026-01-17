// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.commands.autocommands.AlignToPose;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveBase;

import java.sql.Driver;
import java.time.temporal.TemporalQuery;
import java.util.Arrays;
import java.util.Map;

import com.pathplanner.lib.auto.CommandUtil;

import choreo.trajectory.Trajectory;
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
  
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

 

  public Autos(SwerveBase swerve) {
    autoFactory = new AutoFactory(
      swerve::getPose,
      swerve::resetOdometry,
      swerve::followTrajectory,
      true,
      swerve);
    
    this.swerve = swerve;
    

    
  }

  
  
  public AutoRoutine ModularAuto(){
    AutoRoutine routine = autoFactory.newRoutine("ModularAuto");
    String[] posTargets = getPosTargets();
    
    Pose2d[] fullTrajectory = {};    
    if (posTargets != null && posTargets.length >= 2){
      AutoTrajectory[] trajectories = new AutoTrajectory[posTargets.length - 1];
      


      //load trajectorys based on posTargets
      for(int n = 0; n < trajectories.length; n++){
        trajectories[n] = routine.trajectory(posTargets[n] + " - " + posTargets[n+1]);
        
        Pose2d[] trajectoryPose2dList = trajectories[n].getRawTrajectory().getPoses();
        fullTrajectory = Arrays.copyOf(fullTrajectory, fullTrajectory.length + trajectoryPose2dList.length );
        System.arraycopy(trajectoryPose2dList, 0, fullTrajectory, fullTrajectory.length - trajectoryPose2dList.length , trajectoryPose2dList.length);
      } 
      
      
      //When the routine starts run the first trajectory
      routine.active().onTrue(
        Commands.sequence(
          //trajectories[0].resetOdometry(),
          new AlignToPose(trajectories[0].getInitialPose().get().plus(new Transform2d(new Translation2d(0.4,0),new Rotation2d())), swerve),
          trajectories[0].cmd()
        )
      );
      //go through all trajectorys and run them one after another
      for(int n = 0; n < trajectories.length - 1; n++){
        //if the position target is at the reef wait the scoring time
        trajectories[n].done().onTrue(trajectories[n+1].cmd());
      }
      
      swerve.field.getObject("autoTrajectory").setPoses(fullTrajectory);
      swerve.field.getObject("target").setPose(trajectories[0].getInitialPose().get());
    }
    
    return routine;  
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
 
}
