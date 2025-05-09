package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.PathPlannerUtils;

class RobotContainerTest {
    // Mocks
    private SwerveDriveSubsystem mockDriveSubsystem;
    private VisionSubsystem mockVisionSubsystem;
    private PathPlannerUtils mockPathPlannerUtils;
    private XboxController mockDriverController;
    private Joystick mockJoystick;
    
    // For testing button bindings
    private JoystickButton mockJoystickButton;
    
    // System under test
    private RobotContainer robotContainer;
    
    @BeforeEach
    void setUp() throws Exception {
        // Arrange - Create mocks
        mockDriveSubsystem = mock(SwerveDriveSubsystem.class);
        mockVisionSubsystem = mock(VisionSubsystem.class);
        mockPathPlannerUtils = mock(PathPlannerUtils.class);
        mockDriverController = mock(XboxController.class);
        mockJoystick = mock(Joystick.class);
        mockJoystickButton = mock(JoystickButton.class);
        
        // Mock constructor for JoystickButton
        mockStatic(JoystickButton.class);
        when(new JoystickButton(any(Joystick.class), anyInt())).thenReturn(mockJoystickButton);
        
        // Create a test version of RobotContainer that uses our mocks
        robotContainer = new RobotContainer() {
            @Override
            protected void createSubsystems() {
                driveSubsystem = mockDriveSubsystem;
                visionSubsystem = mockVisionSubsystem;
                pathPlannerUtils = mockPathPlannerUtils;
            }
            
            @Override
            protected void createControllers() {
                driverController = mockDriverController;
                joystick = mockJoystick;
            }
        };
    }
    
    @Test
    void configureButtonBindings_ShouldBindAprilTagCommandsToJoystickButtons() {
        // Arrange
        // This is handled in setUp()
        
        // Act
        robotContainer.configureButtonBindings();
        
        // Assert
        // Verify that JoystickButton.onTrue was called for each AprilTag button
        for (int i = 0; i < JoystickConstants.APRILTAG_BUTTONS.length; i++) {
            int buttonIndex = JoystickConstants.APRILTAG_BUTTONS[i];
            int tagId = i + 1;
            
            verify(new JoystickButton(mockJoystick, buttonIndex));
            verify(mockJoystickButton).onTrue(any(DriveToAprilTagCommand.class));
            
            // Verify the command was created with the correct parameters
            verify(new DriveToAprilTagCommand(
                mockDriveSubsystem,
                mockVisionSubsystem,
                mockPathPlannerUtils,
                tagId,
                VisionConstants.APRILTAG_APPROACH_DISTANCE_METERS,
                VisionConstants.LIMELIGHT_FRONT_NAME
            ));
        }
    }
    
    @Test
    void getDriveSubsystem_ShouldReturnDriveSubsystem() {
        // Act
        SwerveDriveSubsystem result = robotContainer.getDriveSubsystem();
        
        // Assert
        assertEquals(mockDriveSubsystem, result);
    }
    
    @Test
    void getAutonomousCommand_ShouldReturnSelectedCommand() {
        // Arrange
        Command mockAutoCommand = mock(Command.class);
        when(mockPathPlannerUtils.getAutoChooser().getSelected()).thenReturn(mockAutoCommand);
        
        // Act
        Command result = robotContainer.getAutonomousCommand();
        
        // Assert
        assertEquals(mockAutoCommand, result);
    }
    
    @Test
    void modifyAxis_WithValueBelowDeadband_ShouldReturnZero() {
        // Arrange
        double input = Constants.DEADBAND / 2.0;
        
        // Act
        double result = RobotContainer.modifyAxis(input);
        
        // Assert
        assertEquals(0.0, result, 0.001);
    }
    
    @Test
    void modifyAxis_WithValueAboveDeadband_ShouldScaleAndSquare() {
        // Arrange
        double input = 0.5; // Above deadband
        double expected = Math.pow((input - Constants.DEADBAND) / (1.0 - Constants.DEADBAND), 2);
        
        // Act
        double result = RobotContainer.modifyAxis(input);
        
        // Assert
        assertEquals(expected, result, 0.001);
    }
    
    @Test
    void modifyAxis_WithNegativeValueBelowDeadband_ShouldReturnZero() {
        // Arrange
        double input = -Constants.DEADBAND / 2.0;
        
        // Act
        double result = RobotContainer.modifyAxis(input);
        
        // Assert
        assertEquals(0.0, result, 0.001);
    }
    
    @Test
    void modifyAxis_WithNegativeValueAboveDeadband_ShouldScaleAndSquareAndPreserveSign() {
        // Arrange
        double input = -0.5; // Below negative deadband
        double expected = -Math.pow((Math.abs(input) - Constants.DEADBAND) / (1.0 - Constants.DEADBAND), 2);
        
        // Act
        double result = RobotContainer.modifyAxis(input);
        
        // Assert
        assertEquals(expected, result, 0.001);
    }
}