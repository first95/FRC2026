// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double NEO_FREE_SPEED = 5676; // RPM
    public static final double NEO_STALL_TORQUE = 3.75; // N * m
    public static final double NEO_550_FREE_SPEED = 11000; // RPM
  public static final double SPARK_VELOCITY_RESPONSE_LOOP = 0.11042; // 110.42ms
  public static final double VORTEX_FREE_SPEED = 6784; // RPM
  public static final double VORTEX_STALL_TORQUE = 3.6; // N * m

    public static final double GRAVITY = 9.81; // m/s/s
    public static final double FIELD_WIDTH = 8.069263;
    public static final double FIELD_LENGTH = 16.541052;

  public static final double LOOP_CYCLE = 0.02; // 20m

  public static final double ROBOT_MASS = 100;
  public static final double MANIPULATOR_MASS = 0;
  public static final double CHASSIS_MASS = ROBOT_MASS - MANIPULATOR_MASS;
  public static final double ARM_Y_POS = 0;
  public static final Translation3d CHASSIS_CG = new Translation3d(
    0,
    0,
    0.15);
    public static final class Drivebase {
        public static final int DEBUG_FLAG = 0b1;
        // Hold time on motor brakes when disabled
        public static final int DISABLED_BREAK_TIMEOUT = 250; // cycles

        public static final int SWERVE_MODULE_CURRENT_LIMIT = 50;

        public static final double HEADING_TOLERANCE = Math.toRadians(1);

        // Motor and encoder inversions
        public static final boolean ABSOLUTE_ENCODER_INVERT = true;
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;
        public static final boolean INVERT_GYRO = false;

        // Module locations, in meters, as distances to the center of the robot.
        // Positive x is torwards the robot front, and +y is torwards robot left.
        public static final double FRONT_LEFT_X = Units.inchesToMeters(23.5/2);
        public static final double FRONT_LEFT_Y = Units.inchesToMeters(23.5/2);
        public static final double FRONT_RIGHT_X = Units.inchesToMeters(23.5/2);
        public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-23.5/2);
        public static final double BACK_LEFT_X = Units.inchesToMeters(-23.5/2);
        public static final double BACK_LEFT_Y = Units.inchesToMeters(23.5/2);
        public static final double BACK_RIGHT_X = Units.inchesToMeters(-23.5/2);
        public static final double BACK_RIGHT_Y = Units.inchesToMeters(-23.5/2);

        public static final Translation2d[] MODULE_LOCATIONS = {
                new Translation2d(Drivebase.FRONT_LEFT_X, Drivebase.FRONT_LEFT_Y),
                new Translation2d(Drivebase.FRONT_RIGHT_X, Drivebase.FRONT_RIGHT_Y),
                new Translation2d(Drivebase.BACK_LEFT_X, Drivebase.BACK_LEFT_Y),
                new Translation2d(Drivebase.BACK_RIGHT_X, Drivebase.BACK_RIGHT_Y)
        };

        // IMU Mounting. CCW Positive
        public static final double IMU_MOUNT_YAW = 180;
        public static final double IMU_MOUNT_PITCH = 0;
        public static final double IMU_MOUNT_ROLL = 0;

        // Drivetrain limitations
        public static final double MAX_SPEED = (VORTEX_FREE_SPEED * Units.inchesToMeters(3 * Math.PI)) / (60 * 4.71); // meters per second NOT A LIMIT!!! DO NOT TOUCH!!!
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y); // rad/s
        // Theoretical max acceleration should be as follows:
        // (NEO stall torque * module gearing * number of modules) / (wheel radius *
        // robot mass) = m/s/s
        // (2.6 * 6.75 * 4) / (Units.inchesToMeters(2) * ROBOT_MASS)
        // However, the drive is traction-limited, so the max accelration is actually
        // (wheel coefficient of friction * gravity)
        public static final double MAX_ACCELERATION = 1 * GRAVITY; // COF is 1.1 but careful
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y);
        // max speed (RPM) / gear ratio, convert to deg/min, divide by 60 for deg/s
        public static final double MAX_MODULE_ANGULAR_SPEED = Units.rotationsToDegrees(NEO_550_FREE_SPEED * 7 / 372)/ 60; // deg/sk

    // Currently does nothing
    public static final double ANGULAR_ACCELERATION_LIMIT = 100;
    public static final double ANGULAR_VELOCITY_LIMIT = 5;

        // Robot control gains
        public static final double HEADING_KP = 6;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0.095;

    public static final double HEADING_MIN_ANGULAR_CONTROL_EFFORT = 0.0005; // rad/s— Prevent oscillation by cancelling rotational commands less than this

        public static final Translation2d CENTEROFROTATION = new Translation2d(0,0);
        // Swerve base kinematics object
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

    public static final double SKEW_CORRECTION_FACTOR = 0;

        // Module PIDF gains
        public static final double MODULE_KP = 0.06;
        public static final double MODULE_KI = 0.0;
        public static final double MODULE_KD = 0.0005;
        public static final double MODULE_IZ = 0;
        public static final double MODULE_KF = 0;
        // Volt * seconds / degree. Equal to (maxVolts) / ((degreesPerRotation) *
        // (maxMotorSpeedRPM / gearRatio) * (minutesPerSecond))
        public static final double MODULE_KV = 12 / MAX_MODULE_ANGULAR_SPEED;

        public static final double VELOCITY_KP = 0.07;
        public static final double VELOCITY_KI = 0.0; // Leave all of these zero to disable them
        public static final double VELOCITY_KD = 0;
        public static final double VELOCITY_IZ = 0;
        public static final double VELOCITY_KF = 0;

        public static final double CURRENT_KP = 0;
        public static final double CURRENT_KI = 0.2;
        public static final double CURRENT_KD = 0;
        public static final double CURRENT_MOVING_AVERAGE_SAMPLES = 1;

        public static final int NUM_MODULES = 4;

        // Drive feedforward gains
        public static final double KS = 0.12392; // Volts
        public static final double KV = 2.0891; // Volt-seconds per meter (max voltage divided by max speed)
        public static final double KA = 0.26159;///0.26159; // Volt-seconds^2 per meter (max voltage/ divided by max accel)
        public static final double KG = (KA / KV);

        // Encoder conversion values. Drive converts motor rotations to linear wheel
        // distance
        // and steering converts motor rotations to module azimuth
        public static final double METERS_PER_MOTOR_ROTATION = (Math.PI * Units.inchesToMeters(3)) / 4.71;
        // Calculation: 3in diameter wheels * pi [circumfrence] / gear ratio
        public static final double DEGREES_PER_STEERING_ROTATION = 360;
        // degrees per rotation / gear ratio between module and motor

        // Module specific constants'
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final double ANGLE_OFFSET = 360-156.7;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(0, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, FRONT_LEFT_X, FRONT_LEFT_Y);
        }

        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final double ANGLE_OFFSET = 360-25.6;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(1, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, FRONT_RIGHT_X, FRONT_RIGHT_Y);
        }

        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final double ANGLE_OFFSET = 360-130.6;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(2, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, BACK_LEFT_X, BACK_LEFT_Y);
        }

        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final double ANGLE_OFFSET = 180-24.96;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(3, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, BACK_RIGHT_X, BACK_RIGHT_Y);
        }

        public static final int PIGEON = 30;

    }
    public static final class ShooterConstants{
        public static final int TOPROLLER_ID = 15;

        public static final boolean TOPROLLERINVERTED = true;
        public static final boolean TOPROLLERMOTOR2INVERTED = true;

        public static final int TOPROLLER_SMARTCURRENTLIMIT = 80;

        public static final boolean SHOOTER_ENCODER_POSITION_ALWAYS_ON = false;
        public static final boolean SHOOTER_VELOCITY_ALWAYS_ON = true;
        public static final int SHOOTER_VELOCITY_PERIOD = 20;
        public static final boolean SHOOTER_INVERTED = false;
        public static final int SHOOTEROUTPUTCURRENT_PERIOD = 20;

        public static final double TOPROLLER_KP = 0.00005;
        public static final double TOPROLLER_KI = 0;
        public static final double TOPROLLER_KD = 0.0;
        public static final double TOPROLLER_KS = 0.03;//0.0340016;
        public static final double TOPROLLER_KV = 0.00180762;
        public static final double TOPROLLER_KA = 0;

        public static final int BOTTOMROLLER_ID = 12;
        public static final int BOTTOMROLLERMOTOR2_ID = 16;


        public static final boolean BOTTOMROLLER2INVERTED = false;
        public static final boolean BOTTOMROLLERINVERTED = true;

        public static final int BOTTOMROLLER_SMARTCURRENTLIMIT = 80;

        public static final double BOTTOMROLLER_KP = 0.0003;
        public static final double BOTTOMROLLER_KI = 0;
        public static final double BOTTOMROLLER_KD = 0.0;
        public static final double BOTTOMROLLER_KS = -0.01;//-0.00740034;
        public static final double BOTTOMROLLER_KV = 0.00181521;
        public static final double BOTTOMROLLER_KA = 0;

        public static final double BOTTOMROLLER2_KP = 0.0003;
        public static final double BOTTOMROLLER2_KI = 0;
        public static final double BOTTOMROLLER2_KD = 0.0;
        public static final double BOTTOMROLLER2_KS = -0.01;//-0.00740034;
        public static final double BOTTOMROLLER2_KV = 0.00181521;
        public static final double BOTTOMROLLER2_KA = 0;

        public static final double TOPROLLER_IDLE_SPEED = 0;
        public static final double BOTTOMROLLER_IDLE_SPEED = 0;

        public static final double TOPROLLER_JUGGLING_SPEED = 400*2;
        public static final double BOTTOMROLLER_JUGGLING_SPEED = 400;

        public static final double MANUALSHOOTSPEED = 2;

        public static final double INDEXER_IDLE_SPEED = 0;
        public static final double INDEXER_EJECT_SPEED = -6000;
        public static final double INDEXER_INJECT_SPEED = 6000;
        public static final double INDEXINGSPEED = 3000;
        public static final double INDEXER_CLEARSHOOTERSPEED = 0;
        public static final double INDEXERPREPSPEED = 0;

        public static final double INDEXER_KP = 0.000001;
        public static final double INDEXER_KI = 0;
        public static final double INDEXER_KD = 0.0;
        public static final double INDEXER_KS = 0.087708;
        public static final double INDEXER_KV = 0.00209302;

        public static final double INDEXER_PREPINDEXER_CURRENT_THRESHOLD = 0;


        public static final double TOPROLLER_CLEARSHOOTER_CURRENT_THRESHOLD = 0;
        public static final double BOTTOMROLLER_CLEARSHOOTER_CURRENT_THRESHOLD = 0;

        public static final double SHOOTERTOLERANCE = 400;
        public static final double SHOOTING_HEADING_TOLERANCE = Math.toRadians(2);

        public static final int INDEXER_ID = 14;

        public static final boolean INDEXER_INVERTED = false;

        public static final int INDEXER_SMARTCURRENTLIMIT = 80;

        public static final int INDEXER_FAULTS_PERIOD_MS = 20;
        public static final int INDEXER_POSITION_PERIOD = 60000;
        public static final int INDEXER_VELOCITY_PERIOD = 20;
        public static final int INDEXER_OUTPUT_CURRENT_PERIOD = 20;

        public static final Translation3d SHOOTERLOCATION = new Translation3d(
            -0.1390345,
            0,
            0.508637
        );
        public static final Transform2d SHOOTERTRANSFORM = new Transform2d(SHOOTERLOCATION.toTranslation2d(),new Rotation2d(1,0));
        public static final Rotation2d SHOOTER_EXIT_ANGLE = Rotation2d.fromDegrees(90-32.8);

        public static final int N_NEWTONLINERIZATIONS = 20;

        public static final double VELOCITY_to_RPM_INTERPOLATIONSLOPE = 193.65429;
        public static final double VELOCITY_to_RPM_INTERPOLATIONINTERCEPT = 667.36514;


        public static final double RPMOFFSET_INCREMENT = 20; 

        public static final double MINRANGE = 2.62;
        public static final double MAXRANGE = Units.inchesToMeters(170);


    }
    public static final class IntakeConstants{
        public static final int MOTOR1_ID = 17;

        public static final double KP = 0.00001;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = -2.62194;
        public static final double KV = 0.00261407;
        public static final double KA = 0;
        public static final boolean MOTOR1_INVERTED = false;

        public static final int SMARTCURRENTLIMIT = 80;

        public static final int FAULTSPERIOD = 20;
        public static final int OUTPUT_CURRENT_PERIOD = 20;

        public static final int MOTOR2_ID = 18;

        public static final boolean MOTOR2_INVERTED = true;

        public static final int AGITATOR1_ID = 10;

        public static final boolean AGITATORINVERTED = true;

        public static final int AGITATOR_SMARTCURRENTLIMIT = 30;

        public static final int AGITATOR2_ID = 11;

        public static final double INTAKEIDLESPEED = 0;
        public static final double INTAKEAIMINGSPEED = 0;
        public static final double SHOOTINGSPEED = 4000;// 2500;
        public static final double INTAKINGSPEED = 4000;
        public static final double EJECTRAWSPEED = -1;
        public static final double INJECTRAWSPEED = 1;

        public static final double AGITATOR1IDLESPEED = 0;
        public static final double AGITATOR1AIMINGSPEED = 0;
        public static final double AGITATOR1INTAKINGSPEED = 0.8;
        public static final double AGITATOR1SHOOTINGSPEED = 0.8;

        public static final double AGITATOR2IDLESPEED = 0;
        public static final double AGITATOR2AIMINGSPEED = 0;
        public static final double AGITATOR2INTAKINGSPEED = 0.8;
        public static final double AGITATOR2SHOOTINGSPEED = 0.8;

        
       
    }

    public static final class ClimberConstants{
        public static final int MOTOR1_ID = 19;

        public static final int MOTOR2_ID = 20;

        public static final boolean INVERTED = true;

        public static final int SMARTCURRENTLIMIT = 80;

        public static final double CLIMBINGSPEED = 1;


    }
    public static final class Vision {
        public static final int DEBUG_FLAG = 0b10;

        public static final int APRILTAG_PIPELINE_NUMBER = 0;
        public static final String BOW_LIMELIGHT_NAME = "limelight-bow";
        public static final String STERN_LIMELIGHT_NAME = "limelight-stern";

        @SuppressWarnings("unused")
        private static final int BOW_IP = 15; // Git-tracked notepad
        @SuppressWarnings("unused")
        private static final int STERN_IP = 14;

        public static final double POSE_ERROR_TOLERANCE = 1;
        public static final double ANGULAR_ERROR_TOLERANCE = Math.toRadians(3);
        public static final int LOOP_CYCLES_BEFORE_RESET = 20;

        public static final double ODOMETRY_TRANSLATIONAL_STD_DEV = 0.01; // Meters and radians
        public static final double ODOMETRY_ANGULAR_STD_DEV = 0.0002;

        public static final double XY_STD_DEV_COEFFICIENT = 0.007;//0.01;
        public static final double ANG_STD_DEV_COEFFICIENT = 0.007;//0.01;

        public static final double MAX_ALLOWABLE_Z_ERROR = 0.15; // Meters
    }

    public static final class Auton {
        // Plumbing via SmartDashboard

        public static final String AUTO_ENABLED_KEY = "autoEnabled";
        public static final String AUTO_INTAKE_KEY = "autoIntake";
        public static final String AUTO_SHOOT_KEY = "autoShoot";
        public static final String AUTO_CLIMB_KEY = "autoClimb";
        public static final String USE_AUTO_SHOOT_KEY = "useAutoShoot";
        public static final String AUTO_AIMKEY = "autoAim";

        public static final String TARGET_HEADING_KEY = "targetHeading";

        public static final double TRANSITION_ENDTIME = 130;
        public static final double FIRSTSHIFT_ENDTIME = 105;
        public static final double SECONDSHIFT_ENDTIME = 80;
        public static final double THIRDSHIFT_ENDTIME = 55;
        public static final double FORTHSHIFT_ENDTIME = 30;
        

        public static final double AUTON_STATIONARY_SCORING_WAIT_TIME = 2;
        public static final double AUTON_PRELOADSCORE_WAIT_TIME = 0.6;
        public static final double STARTING_INTAKE_WAIT = 0.2;

        // Trapezoidal drive PID constants
        public static final double DRIVE_ACCELERATION_LIMIT = 2.1; // m/s/s
        public static final double DRIVE_VELOCITY_LIMIT = 6; // m/s
        public static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            DRIVE_VELOCITY_LIMIT, DRIVE_ACCELERATION_LIMIT);
        
        public static final double ANGULAR_ACCELERATION_LIMIT = 1;
        public static final double ANGULAR_VELOCITY_LIMIT = 3;
        public static final TrapezoidProfile.Constraints ANGULAR_CONSTRAINTS = new TrapezoidProfile.Constraints(
            ANGULAR_VELOCITY_LIMIT, ANGULAR_ACCELERATION_LIMIT);
        
        public static final double FOLLOW_TRAJECTORY_KP = 4;
        public static final double FOLLOW_TRAJECTORY_KI = 0;
        public static final double FOLLOW_TRAJECTORY_KD = 0;


        public static final double AUTO_ALIGN_TIMEOUT_TIMEOUT = 1;
        public static final double ALIGN_TO_POSE_KP = 4;
        public static final double ALIGN_TO_POSE_KI = 0;
        public static final double ALIGN_TO_POSE_KD = 0;

        public static final double DRIVE_POSITIONAL_TOLERANCE = 0.01; // m
        
        private static final Map<String, Translation3d> BLUE_MAP = Map.ofEntries(
            //Map.entry("Hub", new Translation3d(2.0828, 0,0)),
            Map.entry("Hub", new Translation3d((4.011153+5.233016)/2,(4.563567+3.502097)/2,1.828804)),
            Map.entry("passingTarget2", new Translation3d(4.629707-1,4.034631/2,1)),
            Map.entry("passingTarget1", new Translation3d(4.629707-1,FIELD_WIDTH- 4.034631/2,1)),
            Map.entry("climb", new Translation3d())
            
        );
        
        // Iterates through every element in the pose map and mirrors them for the red alliance
        private static final Map<String, Translation3d> RED_MAP =
            BLUE_MAP.entrySet().stream().collect(Collectors.toMap(
                entry -> entry.getKey(),
                entry -> 
                    new Translation3d(
                        FIELD_LENGTH - entry.getValue().getX(),
                        FIELD_WIDTH - entry.getValue().getY(),
                        entry.getValue().getZ()
                    )
                ));
        
        public static final Map<Alliance, Map<String, Translation3d>> POSE_MAP = Map.of(
            Alliance.Blue, BLUE_MAP,
            Alliance.Red, RED_MAP
        );

    }
    public static final class OperatorConstants {
        public static final int driveControllerPort = 0;
        public static final int headingControllerPort = 1;
        public static final int operatorControllerPort = 2;
    
    public static final double joystickDeadband = 0.05;
    }
}