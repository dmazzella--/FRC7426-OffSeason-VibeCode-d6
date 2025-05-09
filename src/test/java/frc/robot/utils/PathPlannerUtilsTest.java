package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

class PathPlannerUtilsTest {
    // Mocks
    private SwerveDriveSubsystem mockDriveSubsystem;
    private Command mockPathCommand;
    
    // System under test
    private PathPlannerUtils pathPlannerUtils;
    
    @BeforeEach
    void setUp() {
        // Arrange - Create mocks
        mockDriveSubsystem = mock(SwerveDriveSubsystem.class);
        mockPathCommand = mock(Command.class);
        
        // Mock static methods
        mockStatic(AutoBuilder.class);
        mockStatic(PathPlannerPath.class);
        
        // Create the utility class to test
        pathPlannerUtils = new PathPlannerUtils(mockDriveSubsystem);
    }
    
    @Test
    void driveToPose_ShouldCreatePathAndReturnCommand() {
        // Arrange
        Pose2d targetPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
        PathPlannerPath mockPath = mock(PathPlannerPath.class);
        
        // Mock the static methods
        when(PathPlannerPath.fromPathPoints(anyList(), any(PathConstraints.class), any(Rotation2d.class)))
            .thenReturn(mockPath);
        when(AutoBuilder.followPath(mockPath)).thenReturn(mockPathCommand);
        
        // Act
        Command result = pathPlannerUtils.driveToPose(targetPose);
        
        // Assert
        assertNotNull(result);
        assertEquals(mockPathCommand, result);
        
        // Verify the path was created with the correct parameters
        verify(PathPlannerPath.class);
        PathPlannerPath.fromPathPoints(anyList(), any(PathConstraints.class), eq(targetPose.getRotation()));
        
        // Verify the path constraints
        ArgumentCaptor<PathConstraints> constraintsCaptor = ArgumentCaptor.forClass(PathConstraints.class);
        verify(PathPlannerPath.class).fromPathPoints(anyList(), constraintsCaptor.capture(), any(Rotation2d.class));
        
        PathConstraints capturedConstraints = constraintsCaptor.getValue();
        assertEquals(PathPlannerConstants.PATH_MAX_VELOCITY, capturedConstraints.maxVelocity, 0.001);
        assertEquals(PathPlannerConstants.PATH_MAX_ACCELERATION, capturedConstraints.maxAcceleration, 0.001);
        
        // Verify AutoBuilder was called to follow the path
        verify(AutoBuilder.class).followPath(mockPath);
    }
    
    @Test
    void getAutoChooser_ShouldReturnNonNullChooser() {
        // Act
        SendableChooser<Command> result = pathPlannerUtils.getAutoChooser();
        
        // Assert
        assertNotNull(result);
    }
}