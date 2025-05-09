package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.util.Optional;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.PathPlannerUtils;

class DriveToAprilTagCommandTest {
    // Mocks
    private SwerveDriveSubsystem mockDriveSubsystem;
    private VisionSubsystem mockVisionSubsystem;
    private PathPlannerUtils mockPathPlannerUtils;
    private Command mockPathCommand;
    
    // Constants for testing
    private static final int TEST_TAG_ID = 3;
    private static final double TEST_DISTANCE = 0.1; // 10cm
    private static final String TEST_CAMERA = "limelight-front";
    
    // System under test
    private DriveToAprilTagCommand command;
    
    @BeforeEach
    void setUp() {
        // Arrange - Create mocks
        mockDriveSubsystem = mock(SwerveDriveSubsystem.class);
        mockVisionSubsystem = mock(VisionSubsystem.class);
        mockPathPlannerUtils = mock(PathPlannerUtils.class);
        mockPathCommand = mock(Command.class);
        
        // Create the command to test
        command = new DriveToAprilTagCommand(
            mockDriveSubsystem,
            mockVisionSubsystem,
            mockPathPlannerUtils,
            TEST_TAG_ID,
            TEST_DISTANCE,
            TEST_CAMERA
        );
        
        // Mock static methods in SmartDashboard
        mockStatic(SmartDashboard.class);
    }
    
    @Test
    void initialize_ShouldSetTargetAprilTag() {
        // Arrange - Nothing additional needed
        
        // Act
        command.initialize();
        
        // Assert
        verify(mockVisionSubsystem).setTargetAprilTag(TEST_TAG_ID);
        verify(SmartDashboard.class).putString(eq("Status"), contains("Looking for AprilTag " + TEST_TAG_ID));
    }
    
    @Test
    void execute_WhenTagNotFound_ShouldNotCreatePathCommand() {
        // Arrange
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.empty());
        
        // Act
        command.initialize();
        command.execute();
        
        // Assert
        verify(mockPathPlannerUtils, never()).driveToPose(any(Pose2d.class));
    }
    
    @Test
    void execute_WhenTagFound_ShouldCreateAndInitializePathCommand() {
        // Arrange
        Pose2d tagPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.of(tagPose));
        when(mockPathPlannerUtils.driveToPose(any(Pose2d.class)))
            .thenReturn(mockPathCommand);
        
        // Act
        command.initialize();
        command.execute();
        
        // Assert
        ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
        verify(mockPathPlannerUtils).driveToPose(poseCaptor.capture());
        verify(mockPathCommand).initialize();
        
        // Verify the target pose is calculated correctly (should be 10cm in front of the tag)
        Pose2d capturedPose = poseCaptor.getValue();
        assertNotNull(capturedPose);
        
        // The target pose should be 10cm away from the tag in the direction opposite to the tag's facing
        // Since the tag is at 90 degrees, the robot should be positioned to the right of the tag
        assertEquals(2.0, capturedPose.getX(), 0.001);
        assertEquals(3.0 - TEST_DISTANCE, capturedPose.getY(), 0.001);
        
        // The robot should be facing the same direction as the tag
        assertEquals(tagPose.getRotation().getDegrees(), capturedPose.getRotation().getDegrees(), 0.001);
        
        verify(SmartDashboard.class).putString(eq("Status"), contains("Found AprilTag " + TEST_TAG_ID));
    }
    
    @Test
    void execute_WhenTagFoundAndPathCommandCreated_ShouldExecutePathCommand() {
        // Arrange
        Pose2d tagPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.of(tagPose));
        when(mockPathPlannerUtils.driveToPose(any(Pose2d.class)))
            .thenReturn(mockPathCommand);
        
        // Act - First execute to find the tag and create path command
        command.initialize();
        command.execute();
        
        // Reset the mock to verify the next execute call
        reset(mockPathCommand);
        
        // Act - Second execute should run the path command
        command.execute();
        
        // Assert
        verify(mockPathCommand).execute();
    }
    
    @Test
    void end_ShouldStopRobotAndCleanup() {
        // Arrange
        Pose2d tagPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.of(tagPose));
        when(mockPathPlannerUtils.driveToPose(any(Pose2d.class)))
            .thenReturn(mockPathCommand);
        
        // Act - Initialize and execute to create the path command
        command.initialize();
        command.execute();
        
        // Act - End the command
        command.end(false);
        
        // Assert
        verify(mockPathCommand).end(false);
        verify(mockDriveSubsystem).drive(0, 0, 0, true);
        verify(mockVisionSubsystem).clearTargetAprilTag();
        verify(SmartDashboard.class).putString(eq("Status"), contains("Completed navigation"));
    }
    
    @Test
    void end_WhenInterrupted_ShouldIndicateInterruption() {
        // Arrange
        Pose2d tagPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.of(tagPose));
        when(mockPathPlannerUtils.driveToPose(any(Pose2d.class)))
            .thenReturn(mockPathCommand);
        
        // Act - Initialize and execute to create the path command
        command.initialize();
        command.execute();
        
        // Act - End the command with interrupted=true
        command.end(true);
        
        // Assert
        verify(mockPathCommand).end(true);
        verify(mockDriveSubsystem).drive(0, 0, 0, true);
        verify(mockVisionSubsystem).clearTargetAprilTag();
        verify(SmartDashboard.class).putString(eq("Status"), contains("Interrupted navigation"));
    }
    
    @Test
    void isFinished_WhenTagNotFoundAndTimeoutOccurs_ShouldReturnTrue() {
        // Arrange - Setup a command with a very short timeout for testing
        DriveToAprilTagCommand testCommand = new DriveToAprilTagCommand(
            mockDriveSubsystem,
            mockVisionSubsystem,
            mockPathPlannerUtils,
            TEST_TAG_ID,
            TEST_DISTANCE,
            TEST_CAMERA
        ) {
            // Override to simulate timeout
            @Override
            protected boolean isTimedOut() {
                return true;
            }
        };
        
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.empty());
        
        // Act
        testCommand.initialize();
        testCommand.execute();
        boolean result = testCommand.isFinished();
        
        // Assert
        assertTrue(result);
        verify(SmartDashboard.class).putString(eq("Status"), contains("Timeout"));
    }
    
    @Test
    void isFinished_WhenPathCommandFinished_ShouldReturnTrue() {
        // Arrange
        Pose2d tagPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.of(tagPose));
        when(mockPathPlannerUtils.driveToPose(any(Pose2d.class)))
            .thenReturn(mockPathCommand);
        when(mockPathCommand.isFinished()).thenReturn(true);
        
        // Act
        command.initialize();
        command.execute();
        boolean result = command.isFinished();
        
        // Assert
        assertTrue(result);
    }
    
    @Test
    void isFinished_WhenPathCommandNotFinished_ShouldReturnFalse() {
        // Arrange
        Pose2d tagPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
        when(mockVisionSubsystem.getAprilTagPose(TEST_TAG_ID, TEST_CAMERA))
            .thenReturn(Optional.of(tagPose));
        when(mockPathPlannerUtils.driveToPose(any(Pose2d.class)))
            .thenReturn(mockPathCommand);
        when(mockPathCommand.isFinished()).thenReturn(false);
        
        // Act
        command.initialize();
        command.execute();
        boolean result = command.isFinished();
        
        // Assert
        assertFalse(result);
    }
    
    // Helper method to simulate timeout for testing
    private interface TimeoutTestable {
        boolean isTimedOut();
    }
}