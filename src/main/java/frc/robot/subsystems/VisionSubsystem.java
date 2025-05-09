package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    // Network tables for each Limelight
    private final NetworkTable limelightFront;
    private final NetworkTable limelightRight;
    private final NetworkTable limelightBack;
    private final NetworkTable limelightLeft;
    
    // Map of camera names to NetworkTables
    private final Map<String, NetworkTable> cameraMap = new HashMap<>();
    
    // Latest pose estimates from each Limelight
    private Optional<LimelightMeasurement> frontPoseEstimate = Optional.empty();
    private Optional<LimelightMeasurement> rightPoseEstimate = Optional.empty();
    private Optional<LimelightMeasurement> backPoseEstimate = Optional.empty();
    private Optional<LimelightMeasurement> leftPoseEstimate = Optional.empty();
    
    // Target AprilTag ID (if any)
    private int targetAprilTagId = -1;
    
    // Map to store AprilTag poses by ID and camera
    private final Map<String, Map<Integer, Pose2d>> aprilTagPoses = new HashMap<>();
    
    public VisionSubsystem() {
        // Initialize NetworkTables for each Limelight
        limelightFront = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_FRONT_NAME);
        limelightRight = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_RIGHT_NAME);
        limelightBack = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_BACK_NAME);
        limelightLeft = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_LEFT_NAME);
        
        // Populate camera map
        cameraMap.put(VisionConstants.LIMELIGHT_FRONT_NAME, limelightFront);
        cameraMap.put(VisionConstants.LIMELIGHT_RIGHT_NAME, limelightRight);
        cameraMap.put(VisionConstants.LIMELIGHT_BACK_NAME, limelightBack);
        cameraMap.put(VisionConstants.LIMELIGHT_LEFT_NAME, limelightLeft);
        
        // Initialize AprilTag pose maps for each camera
        for (String cameraName : cameraMap.keySet()) {
            aprilTagPoses.put(cameraName, new HashMap<>());
        }
        
        // Set pipeline to AprilTag detection for all Limelights
        setPipeline(limelightFront, VisionConstants.APRILTAG_PIPELINE);
        setPipeline(limelightRight, VisionConstants.APRILTAG_PIPELINE);
        setPipeline(limelightBack, VisionConstants.APRILTAG_PIPELINE);
        setPipeline(limelightLeft, VisionConstants.APRILTAG_PIPELINE);
    }
    
    @Override
    public void periodic() {
        // Update pose estimates from all Limelights
        frontPoseEstimate = getLimelightPoseEstimate(limelightFront, VisionConstants.LIMELIGHT_FRONT_TO_ROBOT);
        rightPoseEstimate = getLimelightPoseEstimate(limelightRight, VisionConstants.LIMELIGHT_RIGHT_TO_ROBOT);
        backPoseEstimate = getLimelightPoseEstimate(limelightBack, VisionConstants.LIMELIGHT_BACK_TO_ROBOT);
        leftPoseEstimate = getLimelightPoseEstimate(limelightLeft, VisionConstants.LIMELIGHT_LEFT_TO_ROBOT);
        
        // Update AprilTag poses
        updateAprilTagPoses();
        
        // Publish data to SmartDashboard
        SmartDashboard.putBoolean("Vision/Front Has Target", frontPoseEstimate.isPresent());
        SmartDashboard.putBoolean("Vision/Right Has Target", rightPoseEstimate.isPresent());
        SmartDashboard.putBoolean("Vision/Back Has Target", backPoseEstimate.isPresent());
        SmartDashboard.putBoolean("Vision/Left Has Target", leftPoseEstimate.isPresent());
        
        if (targetAprilTagId >= 0) {
            boolean tagVisible = isAprilTagVisible(targetAprilTagId);
            SmartDashboard.putBoolean("Vision/Target Tag Visible", tagVisible);
            SmartDashboard.putNumber("Vision/Target Tag ID", targetAprilTagId);
        }
    }
    
    /**
     * Update the AprilTag poses from all cameras
     */
    private void updateAprilTagPoses() {
        for (Map.Entry<String, NetworkTable> entry : cameraMap.entrySet()) {
            String cameraName = entry.getKey();
            NetworkTable table = entry.getValue();
            
            // Get the number of AprilTags detected
            double[] tagIDs = table.getEntry("tid").getDoubleArray(new double[0]);
            
            // Clear previous poses for this camera
            aprilTagPoses.get(cameraName).clear();
            
            // Process each detected tag
            for (int i = 0; i < tagIDs.length; i++) {
                int tagId = (int) tagIDs[i];
                
                // Get the tag pose relative to the camera
                double[] camToTarget = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
                
                if (camToTarget.length >= 6) {
                    // Convert to Pose2d (ignoring Z for now)
                    double x = camToTarget[0];
                    double y = camToTarget[1];
                    double yaw = camToTarget[5];
                    
                    Pose2d tagPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
                    
                    // Store the tag pose
                    aprilTagPoses.get(cameraName).put(tagId, tagPose);
                }
            }
        }
    }
    
    /**
     * Set the target AprilTag ID
     * @param tagId The AprilTag ID to target
     */
    public void setTargetAprilTag(int tagId) {
        this.targetAprilTagId = tagId;
        SmartDashboard.putNumber("Vision/Target Tag ID", tagId);
    }
    
    /**
     * Clear the target AprilTag ID
     */
    public void clearTargetAprilTag() {
        this.targetAprilTagId = -1;
        SmartDashboard.putNumber("Vision/Target Tag ID", -1);
    }
    
    /**
     * Check if an AprilTag is visible from any camera
     * @param tagId The AprilTag ID to check
     * @return True if the tag is visible
     */
    public boolean isAprilTagVisible(int tagId) {
        for (Map<Integer, Pose2d> cameraPoses : aprilTagPoses.values()) {
            if (cameraPoses.containsKey(tagId)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Get the pose of an AprilTag from a specific camera
     * @param tagId The AprilTag ID
     * @param cameraName The camera name
     * @return Optional containing the tag pose, or empty if not visible
     */
    public Optional<Pose2d> getAprilTagPose(int tagId, String cameraName) {
        if (!aprilTagPoses.containsKey(cameraName)) {
            return Optional.empty();
        }
        
        Map<Integer, Pose2d> cameraPoses = aprilTagPoses.get(cameraName);
        if (cameraPoses.containsKey(tagId)) {
            return Optional.of(cameraPoses.get(tagId));
        }
        
        return Optional.empty();
    }
    
    /**
     * Get all visible AprilTags from all cameras
     * @return Map of tag IDs to their poses
     */
    public Map<Integer, Pose2d> getAllVisibleAprilTags() {
        Map<Integer, Pose2d> result = new HashMap<>();
        
        for (Map<Integer, Pose2d> cameraPoses : aprilTagPoses.values()) {
            result.putAll(cameraPoses);
        }
        
        return result;
    }
    
    /**
     * Get all valid vision measurements from the Limelights
     * @return List of vision measurements
     */
    public List<LimelightMeasurement> getAllVisionMeasurements() {
        List<LimelightMeasurement> measurements = new ArrayList<>();
        
        frontPoseEstimate.ifPresent(measurements::add);
        rightPoseEstimate.ifPresent(measurements::add);
        backPoseEstimate.ifPresent(measurements::add);
        leftPoseEstimate.ifPresent(measurements::add);
        
        return measurements;
    }
    
    /**
     * Get the best vision measurement based on the lowest ambiguity
     * @return Optional containing the best measurement, or empty if no valid measurements
     */
    public Optional<LimelightMeasurement> getBestVisionMeasurement() {
        List<LimelightMeasurement> measurements = getAllVisionMeasurements();
        
        if (measurements.isEmpty()) {
            return Optional.empty();
        }
        
        // Find the measurement with the lowest ambiguity (most confident)
        return measurements.stream()
            .min((m1, m2) -> Double.compare(m1.ambiguity(), m2.ambiguity()));
    }
    
    /**
     * Set the pipeline for a Limelight
     * @param table Limelight NetworkTable
     * @param pipeline Pipeline number
     */
    private void setPipeline(NetworkTable table, int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Get pose estimate from a Limelight
     * @param table Limelight NetworkTable
     * @param robotToCamera Transform from robot center to camera
     * @return Optional containing the pose estimate, or empty if no valid target
     */
    private Optional<LimelightMeasurement> getLimelightPoseEstimate(NetworkTable table, Pose2d robotToCamera) {
        NetworkTableEntry tv = table.getEntry("tv");
        
        // Check if Limelight has a valid target (tv = 1)
        if (tv.getDouble(0) < 0.5) {
            return Optional.empty();
        }
        
        // Get botpose data (robot pose in field coordinates)
        double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
        
        if (botpose.length < 6) {
            return Optional.empty();
        }
        
        // Extract pose data
        double x = botpose[0];
        double y = botpose[1];
        double z = botpose[2];
        double roll = botpose[3];
        double pitch = botpose[4];
        double yaw = botpose[5];
        
        // Get additional data for measurement quality
        double ta = table.getEntry("ta").getDouble(0);
        double tx = table.getEntry("tx").getDouble(0);
        double ty = table.getEntry("ty").getDouble(0);
        double tagCount = table.getEntry("tid").getDouble(-1);
        double ambiguity = table.getEntry("ta").getDouble(1.0); // Higher ambiguity = less confident
        
        // Create pose from Limelight data
        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        
        // Create measurement with timestamp and quality metrics
        return Optional.of(new LimelightMeasurement(
            pose,
            Timer.getFPGATimestamp(),
            ambiguity,
            tagCount,
            ta
        ));
    }
    
    /**
     * Record for storing Limelight measurements with quality metrics
     */
    public record LimelightMeasurement(
        Pose2d pose,
        double timestamp,
        double ambiguity,
        double tagCount,
        double targetArea
    ) {}
}