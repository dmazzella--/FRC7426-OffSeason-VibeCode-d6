package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Utility class for PathPlanner integration
 */
public class PathPlannerUtils {
    private final SwerveDriveSubsystem driveSubsystem;
    private final SendableChooser<Command> autoChooser;
    
    /**
     * Create a new PathPlannerUtils
     * 
     * @param driveSubsystem The drive subsystem
     */
    public PathPlannerUtils(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        
        // Configure AutoBuilder for PathPlanner
        configureAutoBuilder();
        
        // Register event markers
        registerEventMarkers();
        
        // Configure path visualization
        configurePathVisualization();
        
        // Create auto chooser
        autoChooser = createAutoChooser();
    }
    
    /**
     * Configure AutoBuilder for PathPlanner
     */
    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            driveSubsystem::getPose, // Pose supplier
            driveSubsystem::resetPose, // Pose consumer
            driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier
            driveSubsystem::drive, // ChassisSpeeds consumer
            PathPlannerConstants.PATH_FOLLOWER_CONFIG, // Path follower config
            () -> {
                // Should path be flipped based on alliance color?
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            driveSubsystem // Subsystem requirement
        );
    }
    
    /**
     * Register event markers for PathPlanner
     */
    private void registerEventMarkers() {
        // Register named commands for event markers
        NamedCommands.registerCommand(PathPlannerConstants.INTAKE_MARKER, 
            new PrintCommand("Intake activated"));
            
        NamedCommands.registerCommand(PathPlannerConstants.SHOOT_MARKER, 
            new PrintCommand("Shooter activated"));
            
        NamedCommands.registerCommand(PathPlannerConstants.STOP_MARKER, 
            new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, true), driveSubsystem));
    }
    
    /**
     * Configure path visualization for PathPlanner
     */
    private void configurePathVisualization() {
        // Add a logging callback for visualizing paths in the dashboard
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                // Log the active path to the field
                // This would typically use a Field2d object from the drive subsystem
            }
        );
        
        // Log target poses during path following
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
                // Log the target pose to the field
            }
        );
    }
    
    /**
     * Create auto chooser for PathPlanner
     * 
     * @return SendableChooser for autonomous routines
     */
    private SendableChooser<Command> createAutoChooser() {
        // Create the chooser
        SendableChooser<Command> chooser = new SendableChooser<>();
        
        // Add options to the chooser
        chooser.setDefaultOption("No Auto", new InstantCommand());
        
        // Add autos from PathPlanner
        try {
            // Example autos - these would be created in PathPlanner
            chooser.addOption("Simple Path", new PathPlannerAuto("SimplePath"));
            chooser.addOption("Complex Path", new PathPlannerAuto("ComplexPath"));
            chooser.addOption("Two Piece Auto", new PathPlannerAuto("TwoPieceAuto"));
        } catch (Exception e) {
            System.err.println("Error loading PathPlanner autos: " + e.getMessage());
        }
        
        return chooser;
    }
    
    /**
     * Get the auto chooser
     * 
     * @return SendableChooser for autonomous routines
     */
    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }
    
    /**
     * Load a path from PathPlanner
     * 
     * @param pathName Name of the path
     * @return Command to follow the path
     */
    public Command loadPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            System.err.println("Error loading path " + pathName + ": " + e.getMessage());
            return new PrintCommand("Failed to load path: " + pathName);
        }
    }
    
    /**
     * Create a command to drive to a pose
     * 
     * @param pose Target pose
     * @return Command to drive to the pose
     */
    public Command driveToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
            pose,
            PathPlannerConstants.PATH_FOLLOWER_CONFIG.maxModuleSpeed,
            0.0, // Goal end velocity
            0.0  // Rotation delay distance
        );
    }
}