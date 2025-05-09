package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Example command that creates and follows a path programmatically
 */
public class ExamplePathCommand extends Command {
    private final SwerveDriveSubsystem driveSubsystem;
    private Command pathCommand;
    
    public ExamplePathCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // Create a path programmatically
        PathPlannerPath path = createExamplePath();
        
        // Create a command to follow the path
        pathCommand = AutoBuilder.followPath(path);
        
        // Initialize the path command
        pathCommand.initialize();
    }
    
    @Override
    public void execute() {
        // Execute the path command
        pathCommand.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        // End the path command
        pathCommand.end(interrupted);
    }
    
    @Override
    public boolean isFinished() {
        // Check if the path command is finished
        return pathCommand.isFinished();
    }
    
    /**
     * Create an example path programmatically
     * 
     * @return PathPlannerPath
     */
    private PathPlannerPath createExamplePath() {
        // Define waypoints
        var waypoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90)),
            new Pose2d(1.0, 3.0, Rotation2d.fromDegrees(180)),
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(270))
        );
        
        // Create path with constraints
        return new PathPlannerPath(
            waypoints,
            new PathConstraints(
                PathPlannerConstants.PATH_MAX_VELOCITY,
                PathPlannerConstants.PATH_MAX_ACCELERATION,
                2 * Math.PI, // Max angular velocity
                4 * Math.PI  // Max angular acceleration
            ),
            new Rotation2d() // Goal end rotation
        );
    }
    
    /**
     * Create a command that follows a complex path with events
     * 
     * @param driveSubsystem The drive subsystem
     * @return Command to follow the path
     */
    public static Command createComplexPathCommand(SwerveDriveSubsystem driveSubsystem) {
        return Commands.sequence(
            // Reset pose to starting position
            Commands.runOnce(() -> driveSubsystem.resetPose(new Pose2d(1.0, 1.0, new Rotation2d()))),
            
            // Print start message
            new PrintCommand("Starting complex path"),
            
            // Follow the path
            new ExamplePathCommand(driveSubsystem),
            
            // Print completion message
            new PrintCommand("Path completed")
        );
    }
}