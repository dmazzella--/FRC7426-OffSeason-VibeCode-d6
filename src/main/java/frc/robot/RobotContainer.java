package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.PathPlannerUtils;

public class RobotContainer {
  // Subsystems
  private final SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  
  // PathPlanner utilities
  private final PathPlannerUtils pathPlannerUtils;
  
  // Auto chooser
  private final SendableChooser<Command> autoChooser;
  
  // Controllers
  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

  public RobotContainer() {
    // Connect vision to drive subsystem
    driveSubsystem.setVisionSubsystem(visionSubsystem);
    
    // Initialize PathPlanner utilities
    pathPlannerUtils = new PathPlannerUtils(driveSubsystem);
    
    // Get auto chooser
    autoChooser = pathPlannerUtils.getAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Configure button bindings
    configureButtonBindings();
    
    // Set default command for the drive subsystem
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
      driveSubsystem,
      () -> -modifyAxis(driverController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driverController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driverController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
  }

  private void configureButtonBindings() {
    // Reset gyro with Y button
    new JoystickButton(driverController, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> driveSubsystem.resetPose(driveSubsystem.getPose().withRotation(new edu.wpi.first.math.geometry.Rotation2d()))));
    
    // Toggle vision-assisted pose estimation with X button
    new JoystickButton(driverController, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> {
        boolean currentState = SmartDashboard.getBoolean("Drive/Vision Enabled", true);
        driveSubsystem.setVisionEnabled(!currentState);
      }));
    
    // Configure joystick buttons for AprilTag navigation
    configureAprilTagButtons();
  }
  
  /**
   * Configure joystick buttons for AprilTag navigation
   */
  private void configureAprilTagButtons() {
    // Buttons 1-8 drive to AprilTags 1-8
    for (int i = 0; i < JoystickConstants.APRILTAG_BUTTONS.length; i++) {
      int buttonIndex = JoystickConstants.APRILTAG_BUTTONS[i];
      int tagId = i + 1; // AprilTag IDs 1-8
      
      new JoystickButton(joystick, buttonIndex)
        .onTrue(new DriveToAprilTagCommand(
          driveSubsystem,
          visionSubsystem,
          pathPlannerUtils,
          tagId,
          VisionConstants.APRILTAG_APPROACH_DISTANCE_METERS,
          VisionConstants.LIMELIGHT_FRONT_NAME // Use front camera for all AprilTag navigation
        ));
    }
    
    // Add status display for joystick connection
    SmartDashboard.putBoolean("Joystick Connected", joystick.isConnected());
  }

  public Command getAutonomousCommand() {
    // Return the selected autonomous command from the chooser
    return autoChooser.getSelected();
  }
  
  public SwerveDriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
  
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.DEADBAND);
    
    // Square the axis
    value = Math.copySign(value * value, value);
    
    return value;
  }
  
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}