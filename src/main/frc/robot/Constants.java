package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  // Controller
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int JOYSTICK_PORT = 1;
  public static final double DEADBAND = 0.1;
  
  // Swerve Drive
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
  public static final double DRIVE_GEAR_RATIO = 8.14; // SDS MK4i L2
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0; // SDS MK4i
  
  // Conversion factors
  public static final double DRIVE_POSITION_CONVERSION = (WHEEL_DIAMETER_METERS * Math.PI) / (DRIVE_GEAR_RATIO * 2048.0);
  public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION * 10.0; // Per 100ms to per second
  public static final double TURN_POSITION_CONVERSION = 2.0 * Math.PI / (TURN_GEAR_RATIO * 2048.0);
  
  // Robot dimensions
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21.73);
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(21.73);
  
  // Module positions relative to the center of the robot
  public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
  public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
  public static final Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
  public static final Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
  
  // Motor and encoder IDs
  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
  public static final int FRONT_LEFT_TURN_MOTOR_ID = 2;
  public static final int FRONT_LEFT_ENCODER_ID = 9;
  public static final double FRONT_LEFT_ENCODER_OFFSET = 0.0; // Degrees
  
  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
  public static final int FRONT_RIGHT_TURN_MOTOR_ID = 4;
  public static final int FRONT_RIGHT_ENCODER_ID = 10;
  public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.0; // Degrees
  
  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
  public static final int BACK_LEFT_TURN_MOTOR_ID = 6;
  public static final int BACK_LEFT_ENCODER_ID = 11;
  public static final double BACK_LEFT_ENCODER_OFFSET = 0.0; // Degrees
  
  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
  public static final int BACK_RIGHT_TURN_MOTOR_ID = 8;
  public static final int BACK_RIGHT_ENCODER_ID = 12;
  public static final double BACK_RIGHT_ENCODER_OFFSET = 0.0; // Degrees
  
  // Speed limits
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2.0 * Math.PI;
  
  // PID configuration class
  public static class PIDConfig {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    
    public PIDConfig(double kP, double kI, double kD, double kF) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kF = kF;
    }
    
    @Override
    public String toString() {
      return String.format("P: %.4f, I: %.4f, D: %.4f, F: %.4f", kP, kI, kD, kF);
    }
  }
  
  // Default PID values
  public static final PIDConfig DEFAULT_DRIVE_PID = new PIDConfig(0.1, 0.0, 0.0, 0.05);
  public static final PIDConfig DEFAULT_TURN_PID = new PIDConfig(0.6, 0.0, 0.01, 0.0);
  
  // Vision Constants
  public static final class VisionConstants {
    // Limelight network table names
    public static final String LIMELIGHT_FRONT_NAME = "limelight-front";
    public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
    public static final String LIMELIGHT_BACK_NAME = "limelight-back";
    public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
    
    // Pipeline indices
    public static final int APRILTAG_PIPELINE = 0;
    public static final int RETROREFLECTIVE_PIPELINE = 1;
    
    // Limelight mounting positions relative to robot center
    // These are the transforms from robot center to each camera
    public static final Pose2d LIMELIGHT_FRONT_TO_ROBOT = new Pose2d(
        new Translation2d(WHEEL_BASE_METERS / 2.0 + 0.1, 0),
        Rotation2d.fromDegrees(0)
    );
    
    public static final Pose2d LIMELIGHT_RIGHT_TO_ROBOT = new Pose2d(
        new Translation2d(0, -TRACK_WIDTH_METERS / 2.0 - 0.1),
        Rotation2d.fromDegrees(-90)
    );
    
    public static final Pose2d LIMELIGHT_BACK_TO_ROBOT = new Pose2d(
        new Translation2d(-WHEEL_BASE_METERS / 2.0 - 0.1, 0),
        Rotation2d.fromDegrees(180)
    );
    
    public static final Pose2d LIMELIGHT_LEFT_TO_ROBOT = new Pose2d(
        new Translation2d(0, TRACK_WIDTH_METERS / 2.0 + 0.1),
        Rotation2d.fromDegrees(90)
    );
    
    // Standard deviations for vision measurements (how much to trust the vision)
    // Increase these values to trust vision less, decrease to trust vision more
    public static final double VISION_STD_DEV_X = 0.5;
    public static final double VISION_STD_DEV_Y = 0.5;
    public static final double VISION_STD_DEV_THETA = 0.5;
    
    // AprilTag constants
    public static final double APRILTAG_APPROACH_DISTANCE_METERS = 0.10; // 10 centimeters
  }
  
  // PathPlanner Constants
  public static final class PathPlannerConstants {
    // PID constants for path following
    public static final double PATH_TRANSLATION_P = 5.0;
    public static final double PATH_TRANSLATION_I = 0.0;
    public static final double PATH_TRANSLATION_D = 0.0;
    
    public static final double PATH_ROTATION_P = 5.0;
    public static final double PATH_ROTATION_I = 0.0;
    public static final double PATH_ROTATION_D = 0.0;
    
    // Max module speed during path following
    public static final double PATH_MAX_MODULE_SPEED = MAX_VELOCITY_METERS_PER_SECOND * 0.9;
    
    // Path constraints
    public static final double PATH_MAX_VELOCITY = 3.0; // m/s
    public static final double PATH_MAX_ACCELERATION = 2.0; // m/s²
    
    // Holonomic path follower configuration for PathPlanner
    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(PATH_TRANSLATION_P, PATH_TRANSLATION_I, PATH_TRANSLATION_D), // Translation PID
        new PIDConstants(PATH_ROTATION_P, PATH_ROTATION_I, PATH_ROTATION_D), // Rotation PID
        PATH_MAX_MODULE_SPEED, // Max module speed
        Math.hypot(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0), // Drive base radius (distance from center to furthest module)
        new ReplanningConfig(true, true) // Enable replanning
    );
    
    // Event map for PathPlanner
    public static final String INTAKE_MARKER = "intake";
    public static final String SHOOT_MARKER = "shoot";
    public static final String STOP_MARKER = "stop";
  }
  
  // Joystick Constants
  public static final class JoystickConstants {
    // Number of buttons on the joystick
    public static final int BUTTON_COUNT = 32;
    
    // Button assignments for AprilTag navigation
    public static final int[] APRILTAG_BUTTONS = {1, 2, 3, 4, 5, 6, 7, 8}; // Buttons 1-8 for AprilTags 1-8
  }
}