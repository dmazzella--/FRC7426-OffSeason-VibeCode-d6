package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem.LimelightMeasurement;

class VisionSubsystemTest {
    // Mocks
    private NetworkTableInstance mockNtInstance;
    private NetworkTable mockFrontTable;
    private NetworkTable mockRightTable;
    private NetworkTable mockBackTable;
    private NetworkTable mockLeftTable;
    
    // Mock entries
    private NetworkTableEntry mockTvEntry;
    private NetworkTableEntry mockTaEntry;
    private NetworkTableEntry mockTxEntry;
    private NetworkTableEntry mockTyEntry;
    private NetworkTableEntry mockTidEntry;
    private NetworkTableEntry mockBotposeEntry;
    private NetworkTableEntry mockPipelineEntry;
    private NetworkTableEntry mockTargetposeEntry;
    
    // System under test
    private VisionSubsystem visionSubsystem;
    
    @BeforeEach
    void setUp() {
        // Arrange - Create mocks
        mockNtInstance = mock(NetworkTableInstance.class);
        mockFrontTable = mock(NetworkTable.class);
        mockRightTable = mock(NetworkTable.class);
        mockBackTable = mock(NetworkTable.class);
        mockLeftTable = mock(NetworkTable.class);
        
        // Mock entries
        mockTvEntry = mock(NetworkTableEntry.class);
        mockTaEntry = mock(NetworkTableEntry.class);
        mockTxEntry = mock(NetworkTableEntry.class);
        mockTyEntry = mock(NetworkTableEntry.class);
        mockTidEntry = mock(NetworkTableEntry.class);
        mockBotposeEntry = mock(NetworkTableEntry.class);
        mockPipelineEntry = mock(NetworkTableEntry.class);
        mockTargetposeEntry = mock(NetworkTableEntry.class);
        
        // Mock static methods
        mockStatic(NetworkTableInstance.class);
        mockStatic(Timer.class);
        mockStatic(SmartDashboard.class);
        
        // Setup NetworkTableInstance mock
        when(NetworkTableInstance.getDefault()).thenReturn(mockNtInstance);
        
        // Setup NetworkTable mocks
        when(mockNtInstance.getTable(VisionConstants.LIMELIGHT_FRONT_NAME)).thenReturn(mockFrontTable);
        when(mockNtInstance.getTable(VisionConstants.LIMELIGHT_RIGHT_NAME)).thenReturn(mockRightTable);
        when(mockNtInstance.getTable(VisionConstants.LIMELIGHT_BACK_NAME)).thenReturn(mockBackTable);
        when(mockNtInstance.getTable(VisionConstants.LIMELIGHT_LEFT_NAME)).thenReturn(mockLeftTable);
        
        // Setup NetworkTableEntry mocks for front table (others would be similar)
        when(mockFrontTable.getEntry("tv")).thenReturn(mockTvEntry);
        when(mockFrontTable.getEntry("ta")).thenReturn(mockTaEntry);
        when(mockFrontTable.getEntry("tx")).thenReturn(mockTxEntry);
        when(mockFrontTable.getEntry("ty")).thenReturn(mockTyEntry);
        when(mockFrontTable.getEntry("tid")).thenReturn(mockTidEntry);
        when(mockFrontTable.getEntry("botpose")).thenReturn(mockBotposeEntry);
        when(mockFrontTable.getEntry("pipeline")).thenReturn(mockPipelineEntry);
        when(mockFrontTable.getEntry("targetpose_cameraspace")).thenReturn(mockTargetposeEntry);
        
        // Setup Timer mock
        when(Timer.getFPGATimestamp()).thenReturn(12345.0);
        
        // Create the subsystem to test
        visionSubsystem = new VisionSubsystem() {
            // Override constructor to use our mocks
            {
                // This block replaces the constructor logic
            }
        };
        
        // Use reflection to set the private fields
        try {
            java.lang.reflect.Field limelightFrontField = VisionSubsystem.class.getDeclaredField("limelightFront");
            limelightFrontField.setAccessible(true);
            limelightFrontField.set(visionSubsystem, mockFrontTable);
            
            java.lang.reflect.Field limelightRightField = VisionSubsystem.class.getDeclaredField("limelightRight");
            limelightRightField.setAccessible(true);
            limelightRightField.set(visionSubsystem, mockRightTable);
            
            java.lang.reflect.Field limelightBackField = VisionSubsystem.class.getDeclaredField("limelightBack");
            limelightBackField.setAccessible(true);
            limelightBackField.set(visionSubsystem, mockBackTable);
            
            java.lang.reflect.Field limelightLeftField = VisionSubsystem.class.getDeclaredField("limelightLeft");
            limelightLeftField.setAccessible(true);
            limelightLeftField.set(visionSubsystem, mockLeftTable);
            
            java.lang.reflect.Field cameraMapField = VisionSubsystem.class.getDeclaredField("cameraMap");
            cameraMapField.setAccessible(true);
            Map<String, NetworkTable> cameraMap = (Map<String, NetworkTable>) cameraMapField.get(visionSubsystem);
            cameraMap.put(VisionConstants.LIMELIGHT_FRONT_NAME, mockFrontTable);
            cameraMap.put(VisionConstants.LIMELIGHT_RIGHT_NAME, mockRightTable);
            cameraMap.put(VisionConstants.LIMELIGHT_BACK_NAME, mockBackTable);
            cameraMap.put(VisionConstants.LIMELIGHT_LEFT_NAME, mockLeftTable);
            
        } catch (Exception e) {
            fail("Failed to set up test: " + e.getMessage());
        }
    }
    
    @Test
    void setTargetAprilTag_ShouldSetTargetIdAndUpdateSmartDashboard() {
        // Arrange
        int testTagId = 5;
        
        // Act
        visionSubsystem.setTargetAprilTag(testTagId);
        
        // Assert
        assertEquals(testTagId, getTargetAprilTagId());
        verify(SmartDashboard.class).putNumber(eq("Vision/Target Tag ID"), eq((double) testTagId));
    }
    
    @Test
    void clearTargetAprilTag_ShouldClearTargetIdAndUpdateSmartDashboard() {
        // Arrange - Set a target first
        visionSubsystem.setTargetAprilTag(5);
        
        // Act
        visionSubsystem.clearTargetAprilTag();
        
        // Assert
        assertEquals(-1, getTargetAprilTagId());
        verify(SmartDashboard.class).putNumber(eq("Vision/Target Tag ID"), eq(-1.0));
    }
    
    @Test
    void isAprilTagVisible_WhenTagIsVisible_ShouldReturnTrue() {
        // Arrange
        int testTagId = 3;
        setupMockAprilTagDetection(testTagId);
        
        // Force an update of the AprilTag poses
        visionSubsystem.periodic();
        
        // Act
        boolean result = visionSubsystem.isAprilTagVisible(testTagId);
        
        // Assert
        assertTrue(result);
    }
    
    @Test
    void isAprilTagVisible_WhenTagIsNotVisible_ShouldReturnFalse() {
        // Arrange
        int testTagId = 3;
        int differentTagId = 4;
        setupMockAprilTagDetection(differentTagId);
        
        // Force an update of the AprilTag poses
        visionSubsystem.periodic();
        
        // Act
        boolean result = visionSubsystem.isAprilTagVisible(testTagId);
        
        // Assert
        assertFalse(result);
    }
    
    @Test
    void getAprilTagPose_WhenTagIsVisible_ShouldReturnPose() {
        // Arrange
        int testTagId = 3;
        Pose2d expectedPose = new Pose2d(1.5, 2.5, Rotation2d.fromDegrees(45));
        setupMockAprilTagDetection(testTagId, expectedPose);
        
        // Force an update of the AprilTag poses
        visionSubsystem.periodic();
        
        // Act
        Optional<Pose2d> result = visionSubsystem.getAprilTagPose(testTagId, VisionConstants.LIMELIGHT_FRONT_NAME);
        
        // Assert
        assertTrue(result.isPresent());
        assertEquals(expectedPose.getX(), result.get().getX(), 0.001);
        assertEquals(expectedPose.getY(), result.get().getY(), 0.001);
        assertEquals(expectedPose.getRotation().getDegrees(), result.get().getRotation().getDegrees(), 0.001);
    }
    
    @Test
    void getAprilTagPose_WhenTagIsNotVisible_ShouldReturnEmpty() {
        // Arrange
        int testTagId = 3;
        int differentTagId = 4;
        setupMockAprilTagDetection(differentTagId);
        
        // Force an update of the AprilTag poses
        visionSubsystem.periodic();
        
        // Act
        Optional<Pose2d> result = visionSubsystem.getAprilTagPose(testTagId, VisionConstants.LIMELIGHT_FRONT_NAME);
        
        // Assert
        assertFalse(result.isPresent());
    }
    
    @Test
    void getAprilTagPose_WithInvalidCamera_ShouldReturnEmpty() {
        // Arrange
        int testTagId = 3;
        setupMockAprilTagDetection(testTagId);
        
        // Force an update of the AprilTag poses
        visionSubsystem.periodic();
        
        // Act
        Optional<Pose2d> result = visionSubsystem.getAprilTagPose(testTagId, "invalid-camera");
        
        // Assert
        assertFalse(result.isPresent());
    }
    
    @Test
    void getAllVisibleAprilTags_ShouldReturnAllTags() {
        // Arrange
        setupMockAprilTagDetection(1, new Pose2d(1, 1, new Rotation2d()), mockFrontTable);
        setupMockAprilTagDetection(2, new Pose2d(2, 2, new Rotation2d()), mockRightTable);
        setupMockAprilTagDetection(3, new Pose2d(3, 3, new Rotation2d()), mockBackTable);
        
        // Force an update of the AprilTag poses
        visionSubsystem.periodic();
        
        // Act
        Map<Integer, Pose2d> result = visionSubsystem.getAllVisibleAprilTags();
        
        // Assert
        assertEquals(3, result.size());
        assertTrue(result.containsKey(1));
        assertTrue(result.containsKey(2));
        assertTrue(result.containsKey(3));
    }
    
    @Test
    void getAllVisionMeasurements_WhenNoValidTargets_ShouldReturnEmptyList() {
        // Arrange
        when(mockTvEntry.getDouble(0)).thenReturn(0.0); // No valid target
        
        // Force an update
        visionSubsystem.periodic();
        
        // Act
        List<LimelightMeasurement> result = visionSubsystem.getAllVisionMeasurements();
        
        // Assert
        assertTrue(result.isEmpty());
    }
    
    @Test
    void getAllVisionMeasurements_WithValidTargets_ShouldReturnMeasurements() {
        // Arrange
        setupMockLimelightPoseEstimate(mockFrontTable, true);
        setupMockLimelightPoseEstimate(mockRightTable, true);
        setupMockLimelightPoseEstimate(mockBackTable, false); // No valid target
        setupMockLimelightPoseEstimate(mockLeftTable, true);
        
        // Force an update
        visionSubsystem.periodic();
        
        // Act
        List<LimelightMeasurement> result = visionSubsystem.getAllVisionMeasurements();
        
        // Assert
        assertEquals(3, result.size()); // Should have 3 valid measurements
    }
    
    @Test
    void getBestVisionMeasurement_WhenNoValidTargets_ShouldReturnEmpty() {
        // Arrange
        when(mockTvEntry.getDouble(0)).thenReturn(0.0); // No valid target
        
        // Force an update
        visionSubsystem.periodic();
        
        // Act
        Optional<LimelightMeasurement> result = visionSubsystem.getBestVisionMeasurement();
        
        // Assert
        assertFalse(result.isPresent());
    }
    
    @Test
    void getBestVisionMeasurement_WithValidTargets_ShouldReturnLowestAmbiguity() {
        // Arrange
        setupMockLimelightPoseEstimate(mockFrontTable, true, 0.5); // Ambiguity 0.5
        setupMockLimelightPoseEstimate(mockRightTable, true, 0.2); // Ambiguity 0.2 (best)
        setupMockLimelightPoseEstimate(mockBackTable, true, 0.8); // Ambiguity 0.8
        
        // Force an update
        visionSubsystem.periodic();
        
        // Act
        Optional<LimelightMeasurement> result = visionSubsystem.getBestVisionMeasurement();
        
        // Assert
        assertTrue(result.isPresent());
        assertEquals(0.2, result.get().ambiguity(), 0.001); // Should select the one with lowest ambiguity
    }
    
    @Test
    void periodic_ShouldUpdateSmartDashboard() {
        // Arrange
        setupMockLimelightPoseEstimate(mockFrontTable, true);
        setupMockLimelightPoseEstimate(mockRightTable, false);
        setupMockLimelightPoseEstimate(mockBackTable, true);
        setupMockLimelightPoseEstimate(mockLeftTable, false);
        
        // Act
        visionSubsystem.periodic();
        
        // Assert
        verify(SmartDashboard.class).putBoolean("Vision/Front Has Target", true);
        verify(SmartDashboard.class).putBoolean("Vision/Right Has Target", false);
        verify(SmartDashboard.class).putBoolean("Vision/Back Has Target", true);
        verify(SmartDashboard.class).putBoolean("Vision/Left Has Target", false);
    }
    
    @Test
    void periodic_WithTargetAprilTag_ShouldUpdateTagVisibility() {
        // Arrange
        int testTagId = 3;
        visionSubsystem.setTargetAprilTag(testTagId);
        setupMockAprilTagDetection(testTagId);
        
        // Act
        visionSubsystem.periodic();
        
        // Assert
        verify(SmartDashboard.class).putBoolean("Vision/Target Tag Visible", true);
        verify(SmartDashboard.class).putNumber("Vision/Target Tag ID", testTagId);
    }
    
    // Helper methods
    
    private int getTargetAprilTagId() {
        try {
            java.lang.reflect.Field field = VisionSubsystem.class.getDeclaredField("targetAprilTagId");
            field.setAccessible(true);
            return (int) field.get(visionSubsystem);
        } catch (Exception e) {
            fail("Failed to get targetAprilTagId: " + e.getMessage());
            return -1;
        }
    }
    
    private void setupMockAprilTagDetection(int tagId) {
        setupMockAprilTagDetection(tagId, new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30)));
    }
    
    private void setupMockAprilTagDetection(int tagId, Pose2d pose) {
        setupMockAprilTagDetection(tagId, pose, mockFrontTable);
    }
    
    private void setupMockAprilTagDetection(int tagId, Pose2d pose, NetworkTable table) {
        // Mock tag ID detection
        when(table.getEntry("tid")).thenReturn(mockTidEntry);
        when(mockTidEntry.getDoubleArray(any())).thenReturn(new double[]{tagId});
        
        // Mock tag pose
        when(table.getEntry("targetpose_cameraspace")).thenReturn(mockTargetposeEntry);
        when(mockTargetposeEntry.getDoubleArray(any())).thenReturn(new double[]{
            pose.getX(), pose.getY(), 0.0, 0.0, 0.0, pose.getRotation().getDegrees()
        });
    }
    
    private void setupMockLimelightPoseEstimate(NetworkTable table, boolean hasTarget) {
        setupMockLimelightPoseEstimate(table, hasTarget, 0.5);
    }
    
    private void setupMockLimelightPoseEstimate(NetworkTable table, boolean hasTarget, double ambiguity) {
        // Mock tv (target valid)
        NetworkTableEntry tvEntry = mock(NetworkTableEntry.class);
        when(table.getEntry("tv")).thenReturn(tvEntry);
        when(tvEntry.getDouble(0)).thenReturn(hasTarget ? 1.0 : 0.0);
        
        if (hasTarget) {
            // Mock botpose
            NetworkTableEntry botposeEntry = mock(NetworkTableEntry.class);
            when(table.getEntry("botpose")).thenReturn(botposeEntry);
            when(botposeEntry.getDoubleArray(any())).thenReturn(new double[]{1.0, 2.0, 0.0, 0.0, 0.0, 30.0});
            
            // Mock ta (target area)
            NetworkTableEntry taEntry = mock(NetworkTableEntry.class);
            when(table.getEntry("ta")).thenReturn(taEntry);
            when(taEntry.getDouble(0)).thenReturn(5.0);
            when(taEntry.getDouble(1.0)).thenReturn(ambiguity);
            
            // Mock tx, ty (target x/y)
            NetworkTableEntry txEntry = mock(NetworkTableEntry.class);
            when(table.getEntry("tx")).thenReturn(txEntry);
            when(txEntry.getDouble(0)).thenReturn(2.0);
            
            // Mock tid (tag ID)
            NetworkTableEntry tidEntry = mock(NetworkTableEntry.class);
            when(table.getEntry("tid")).thenReturn(tidEntry);
            when(tidEntry.getDouble(-1)).thenReturn(1.0);
        }
    }
}