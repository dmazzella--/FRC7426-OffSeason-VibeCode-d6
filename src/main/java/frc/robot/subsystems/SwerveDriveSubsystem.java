package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDConfig;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem.LimelightMeasurement;

import java.util.List;

public class SwerveDriveSubsystem extends SubsystemBase {
  // Create IMU (gyro)
  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  
  // Create swerve modules
  private final SwerveModule frontLeft = new SwerveModule(
    Constants.FRONT_LEFT_DRIVE_MOTOR_ID,
    Constants.FRONT_LEFT_TURN_MOTOR_ID,
    Constants.FRONT_LEFT_ENCODER_ID,
    Constants.FRONT_LEFT_ENCODER_OFFSET,
    Constants.DEFAULT_DRIVE_PID,
    Constants.DEFAULT_TURN_PID
  );
  
  private final SwerveModule frontRight = new SwerveModule(
    Constants.FRONT_RIGHT_DRIVE_MOTOR_ID,
    Constants.FRONT_RIGHT_TURN_MOTOR_ID,
    Constants.FRONT_RIGHT_ENCODER_ID,
    Constants.FRONT_RIGHT_ENCODER_OFFSET,
    Constants.DEFAULT_DRIVE_PID,
    Constants.DEFAULT_TURN_PID
  );
  
  private final SwerveModule backLeft = new SwerveModule(
    Constants.BACK_LEFT_DRIVE_MOTOR_ID,
    Constants.BACK_LEFT_TURN_MOTOR_ID,
    Constants.BACK_LEFT_ENCODER_ID,
    Constants.BACK_LEFT_ENCODER_OFFSET,
    Constants.DEFAULT_DRIVE_PID,
    Constants.DEFAULT_TURN_PID
  );
  
  private final SwerveModule backRight = new SwerveModule(
    Constants.BACK_RIGHT_DRIVE_MOTOR_ID,
    Constants.BACK_RIGHT_TURN_MOTOR_ID,
    Constants.BACK_RIGHT_ENCODER_ID,
    Constants.BACK_RIGHT_ENCODER_OFFSET,
    Constants.DEFAULT_DRIVE_PID,
    Constants.DEFAULT_TURN_PID
  );
  
  // Kinematics
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    Constants.FRONT_LEFT_MODULE_POSITION,
    Constants.FRONT_RIGHT_MODULE_POSITION,
    Constants.BACK_LEFT_MODULE_POSITION,
    Constants.BACK_RIGHT_MODULE_POSITION
  );
  
  // Pose estimator (combines odometry and vision)
  private final SwerveDrivePoseEstimator poseEstimator;
  
  // Field visualization
  private final Field2d field2d = new Field2d();
  
  // Vision subsystem reference
  private VisionSubsystem visionSubsystem;
  
  // Flag to enable/disable vision-assisted pose estimation
  private boolean visionEnabled = true;
  
  public SwerveDriveSubsystem() {
    // Reset IMU
    imu.reset();
    
    // Initialize pose estimator with standard deviations for state estimation
    // Higher values = less trust in the measurement
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      },
      new Pose2d() // Initial pose
    );
    
    // Add field visualization to SmartDashboard
    SmartDashboard.putData("Field", field2d);
  }
  
  /**
   * Set the vision subsystem reference
   * @param visionSubsystem The vision subsystem
   */
  public void setVisionSubsystem(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
  }
  
  /**
   * Enable or disable vision-assisted pose estimation
   * @param enabled Whether vision should be used
   */
  public void setVisionEnabled(boolean enabled) {
    this.visionEnabled = enabled;
    SmartDashboard.putBoolean("Drive/Vision Enabled", enabled);
  }
  
  @Override
  public void periodic() {
    // Update pose estimator with latest module positions and gyro angle
    poseEstimator.update(
      getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
    );
    
    // If vision is enabled and vision subsystem is available, add vision measurements
    if (visionEnabled && visionSubsystem != null) {
      List<LimelightMeasurement> visionMeasurements = visionSubsystem.getAllVisionMeasurements();
      
      for (LimelightMeasurement measurement : visionMeasurements) {
        // Calculate standard deviations based on measurement quality
        // Lower ambiguity and higher target area = more trust in the measurement
        double ambiguityFactor = Math.min(5.0, measurement.ambiguity() * 10.0);
        double areaFactor = Math.max(0.5, 5.0 / Math.max(0.1, measurement.targetArea()));
        
        double xStdDev = VisionConstants.VISION_STD_DEV_X * ambiguityFactor * areaFactor;
        double yStdDev = VisionConstants.VISION_STD_DEV_Y * ambiguityFactor * areaFactor;
        double thetaStdDev = VisionConstants.VISION_STD_DEV_THETA * ambiguityFactor * areaFactor;
        
        // Add vision measurement to pose estimator with calculated standard deviations
        poseEstimator.addVisionMeasurement(
          measurement.pose(),
          measurement.timestamp(),
          new double[] {xStdDev, yStdDev, thetaStdDev}
        );
      }
    }
    
    // Get current estimated pose
    Pose2d pose = poseEstimator.getEstimatedPosition();
    
    // Update field visualization
    field2d.setRobotPose(pose);
    
    // Publish pose to SmartDashboard
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
    
    // Publish module states
    SmartDashboard.putNumber("FL Speed", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FR Speed", frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BL Speed", backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BR Speed", backRight.getState().speedMetersPerSecond);
    
    SmartDashboard.putNumber("FL Angle", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("FR Angle", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("BL Angle", backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("BR Angle", backRight.getState().angle.getDegrees());
  }
  
  /**
   * Drive the robot with given velocities
   * 
   * @param xSpeed Forward/backward speed (m/s)
   * @param ySpeed Left/right speed (m/s)
   * @param rotSpeed Rotational speed (rad/s)
   * @param fieldRelative Whether the drive is field-relative
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
      
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Normalize wheel speeds if any exceed the maximum velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
    
    // Set the module states
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }
  
  /**
   * Set the module states directly
   * 
   * @param moduleStates Array of desired module states
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    // Normalize wheel speeds if any exceed the maximum velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
    
    // Set the module states
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }
  
  /**
   * Get the current chassis speeds
   * 
   * @return Current chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    );
  }
  
  /**
   * Get the swerve drive kinematics
   * 
   * @return Swerve drive kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }
  
  /**
   * Update PID values for all swerve modules
   * 
   * @param drivePID New drive PID configuration
   * @param turnPID New turn PID configuration
   */
  public void updatePIDValues(PIDConfig drivePID, PIDConfig turnPID) {
    frontLeft.updatePIDValues(drivePID, turnPID);
    frontRight.updatePIDValues(drivePID, turnPID);
    backLeft.updatePIDValues(drivePID, turnPID);
    backRight.updatePIDValues(drivePID, turnPID);
    
    System.out.println("Updated PID values - Drive: " + drivePID + ", Turn: " + turnPID);
  }
  
  /**
   * Get the robot's rotation
   * 
   * @return Current rotation as Rotation2d
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(imu.getAngle());
  }
  
  /**
   * Get the current estimated pose
   * 
   * @return Current estimated pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  /**
   * Reset the robot's position
   * 
   * @param pose New pose
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      },
      pose
    );
  }
}