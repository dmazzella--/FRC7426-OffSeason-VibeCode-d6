package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.PathPlannerUtils;

/**
 * Command to drive to a position relative to an AprilTag
 */
public class DriveToAprilTagCommand extends Command {
    private final SwerveDriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final PathPlannerUtils pathPlannerUtils;
    private final int tagId;
    private final double distanceFromTag; // in meters
    private final String cameraName;
    
    private Command pathCommand;
    private boolean tagFound = false;
    private final Timer timeoutTimer = new Timer();
    private static final double TIMEOUT_SECONDS = 5.0; // Timeout if tag not found
    
    /**
     * Create a command to drive to a position relative to an AprilTag
     * 
     * @param driveSubsystem The drive subsystem
     * @param visionSubsystem The vision subsystem
     * @param pathPlannerUtils The PathPlanner utilities
     * @param tagId The AprilTag ID to drive to
     * @param distanceFromTag The distance to maintain from the tag (meters)
     * @param cameraName The name of the camera to use
     */
    public DriveToAprilTagCommand(
        SwerveDriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem,
        PathPlannerUtils pathPlannerUtils,
        int tagId,
        double distanceFromTag,
        String cameraName
    ) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.pathPlannerUtils = pathPlannerUtils;
        this.tagId = tagId;
        this.distanceFromTag = distanceFromTag;
        this.cameraName = cameraName;
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        timeoutTimer.reset();
        timeoutTimer.start();
        tagFound = false;
        
        // Set the vision subsystem to look for the specific tag
        visionSubsystem.setTargetAprilTag(tagId);
        
        SmartDashboard.putString("Status", "Looking for AprilTag " + tagId);
    }
    
    @Override
    public void execute() {
        if (!tagFound && timeoutTimer.get() < TIMEOUT_SECONDS) {
            // Try to find the tag
            var tagPose = visionSubsystem.getAprilTagPose(tagId, cameraName);
            
            if (tagPose.isPresent()) {
                tagFound = true;
                SmartDashboard.putString("Status", "Found AprilTag " + tagId + ", navigating");
                
                // Calculate target pose (10cm in front of the tag)
                Pose2d targetPose = calculateTargetPose(tagPose.get());
                
                // Create and start path command
                pathCommand = pathPlannerUtils.driveToPose(targetPose);
                pathCommand.initialize();
            }
        } else if (tagFound && pathCommand != null) {
            // Execute the path command
            pathCommand.execute();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }
        
        // Stop the robot
        driveSubsystem.drive(0, 0, 0, true);
        
        // Reset the vision subsystem
        visionSubsystem.clearTargetAprilTag();
        
        SmartDashboard.putString("Status", interrupted ? 
            "Interrupted navigation to AprilTag " + tagId : 
            "Completed navigation to AprilTag " + tagId);
    }
    
    @Override
    public boolean isFinished() {
        // Finish if timeout occurs without finding the tag
        if (!tagFound && timeoutTimer.get() >= TIMEOUT_SECONDS) {
            SmartDashboard.putString("Status", "Timeout: AprilTag " + tagId + " not found");
            return true;
        }
        
        // Finish if path command is finished
        return tagFound && pathCommand != null && pathCommand.isFinished();
    }
    
    /**
     * Calculate the target pose relative to the AprilTag
     * 
     * @param tagPose The pose of the AprilTag
     * @return The target pose for the robot
     */
    private Pose2d calculateTargetPose(Pose2d tagPose) {
        // Calculate a position in front of the tag
        // The tag's rotation tells us which way it's facing
        Rotation2d tagRotation = tagPose.getRotation();
        
        // Create a transform that moves distanceFromTag meters opposite to the tag's facing direction
        Transform2d transform = new Transform2d(
            new Translation2d(-distanceFromTag, 0), // Move opposite to tag's facing direction
            new Rotation2d() // Keep the same rotation
        );
        
        // Apply the transform to get the target pose
        // This will position the robot distanceFromTag meters in front of the tag, facing the tag
        Pose2d targetPose = tagPose.plus(transform);
        
        // Make the robot face the tag
        targetPose = new Pose2d(
            targetPose.getX(),
            targetPose.getY(),
            tagRotation // Face the same direction as the tag
        );
        
        return targetPose;
    }
}