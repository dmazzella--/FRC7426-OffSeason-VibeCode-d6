package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConfig;
import frc.robot.subsystems.VisionSubsystem.LimelightMeasurement;

class SwerveDriveSubsystemTest {
    // Mocks
    private VisionSubsystem mockVisionSubsystem;
    private SwerveDrivePoseEstimator mockPoseEstimator;
    
    // System under test
    private SwerveDriveSubsystem driveSubsystem;
    
    @BeforeEach
    void setUp() throws Exception {
        // Arrange - Create mocks
        mockVisionSubsystem = mock(VisionSubsystem.class);
        mockPoseEstimator = mock(SwerveDrivePoseEstimator.class);
        
        // Mock static methods
        mockStatic(SmartDashboard.class);
        
        // Create a test version of SwerveDriveSubsystem that uses our mocks
        driveSubsystem = new SwerveDriveSubsystem() {
            @Override
            protected void createModules() {
                // Skip module creation for testing
            }
            
            @Override
            protected void createPoseEstimator() {
                poseEstimator = mockPoseEstimator;
            }
        };
        
        // Set the vision subsystem
        driveSubsystem.setVisionSubsystem(mockVisionSubsystem);
    }
    
    @Test
    void setVisionSubsystem_ShouldStoreReference() {
        // Arrange
        VisionSubsystem newMockVision = mock(VisionSubsystem.class);
        
        // Act
        driveSubsystem.setVisionSubsystem(newMockVision);
        
        // Assert
        assertEquals(newMockVision, driveSubsystem.getVisionSubsystem());
    }
    
    @Test
    void setVisionEnabled_ShouldUpdateSmartDashboard() {
        // Act
        driveSubsystem.setVisionEnabled(true);
        
        // Assert
        verify(SmartDashboard.class).putBoolean("Drive/Vision Enabled", true);
        
        // Act again with false
        driveSubsystem.setVisionEnabled(false);
        
        // Assert
        verify(SmartDashboard.class).putBoolean("Drive/Vision Enabled", false);
    }
    
    @Test
    void periodic_WithVisionEnabled_ShouldAddVisionMeasurements() {
        // Arrange
        driveSubsystem.setVisionEnabled(true);
        
        Pose2d pose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30));
        Pose2d pose2 = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60));
        
        LimelightMeasurement measurement1 = new LimelightMeasurement(pose1, 12345.0, 0.2, 2.0, 5.0);
        LimelightMeasurement measurement2 = new LimelightMeasurement(pose2, 12346.0, 0.3, 1.0, 4.0);
        
        List<LimelightMeasurement> measurements = List.of(measurement1, measurement2);
        
        when(mockVisionSubsystem.getAllVisionMeasurements()).thenReturn(measurements);
        when(SmartDashboard.getBoolean("Drive/Vision Enabled", true)).thenReturn(true);
        
        // Act
        driveSubsystem.periodic();
        
        // Assert
        verify(mockPoseEstimator, times(2)).addVisionMeasurement(any(Pose2d.class), anyDouble(), any());
    }
    
    @Test
    void periodic_WithVisionDisabled_ShouldNotAddVisionMeasurements() {
        // Arrange
        driveSubsystem.setVisionEnabled(false);
        
        Pose2d pose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30));
        LimelightMeasurement measurement1 = new LimelightMeasurement(pose1, 12345.0, 0.2, 2.0, 5.0);
        
        List<LimelightMeasurement> measurements = List.of(measurement1);
        
        when(mockVisionSubsystem.getAllVisionMeasurements()).thenReturn(measurements);
        when(SmartDashboard.getBoolean("Drive/Vision Enabled", true)).thenReturn(false);
        
        // Act
        driveSubsystem.periodic();
        
        // Assert
        verify(mockPoseEstimator, never()).addVisionMeasurement(any(Pose2d.class), anyDouble(), any());
    }
    
    @Test
    void getPose_ShouldReturnPoseFromEstimator() {
        // Arrange
        Pose2d expectedPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30));
        when(mockPoseEstimator.getEstimatedPosition()).thenReturn(expectedPose);
        
        // Act
        Pose2d result = driveSubsystem.getPose();
        
        // Assert
        assertEquals(expectedPose, result);
    }
    
    @Test
    void resetPose_ShouldResetPoseEstimator() {
        // Arrange
        Pose2d newPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));
        
        // Act
        driveSubsystem.resetPose(newPose);
        
        // Assert
        verify(mockPoseEstimator).resetPosition(any(), any(), eq(newPose));
    }
    
    @Test
    void updatePIDValues_ShouldUpdatePIDValues() {
        // Arrange
        PIDConfig drivePID = new PIDConfig(0.1, 0.2, 0.3, 0.4);
        PIDConfig turnPID = new PIDConfig(0.5, 0.6, 0.7, 0.8);
        
        // Act
        driveSubsystem.updatePIDValues(drivePID, turnPID);
        
        // Assert
        // This is mostly a coverage test since we can't easily verify the internal state
        // In a real test, we would verify that the PID values were applied to the controllers
        verify(SmartDashboard.class).putString("Drive/Drive PID", drivePID.toString());
        verify(SmartDashboard.class).putString("Drive/Turn PID", turnPID.toString());
    }
}