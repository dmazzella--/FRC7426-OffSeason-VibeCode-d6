package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.JoystickStatusCommand;
import frc.robot.web.PIDTunerServer;

class RobotTest {
    // Mocks
    private RobotContainer mockRobotContainer;
    private PIDTunerServer mockPidTunerServer;
    private CommandScheduler mockCommandScheduler;
    private Joystick mockJoystick;
    private JoystickStatusCommand mockJoystickStatusCommand;
    
    // System under test
    private Robot robot;
    
    @BeforeEach
    void setUp() {
        // Arrange - Create mocks
        mockRobotContainer = mock(RobotContainer.class);
        mockPidTunerServer = mock(PIDTunerServer.class);
        mockCommandScheduler = mock(CommandScheduler.class);
        mockJoystick = mock(Joystick.class);
        mockJoystickStatusCommand = mock(JoystickStatusCommand.class);
        
        // Mock static methods
        mockStatic(CommandScheduler.class);
        when(CommandScheduler.getInstance()).thenReturn(mockCommandScheduler);
        
        // Mock constructors
        mockStatic(RobotContainer.class);
        when(new RobotContainer()).thenReturn(mockRobotContainer);
        
        mockStatic(PIDTunerServer.class);
        when(new PIDTunerServer(anyInt())).thenReturn(mockPidTunerServer);
        
        mockStatic(Joystick.class);
        when(new Joystick(Constants.JOYSTICK_PORT)).thenReturn(mockJoystick);
        
        mockStatic(JoystickStatusCommand.class);
        when(new JoystickStatusCommand(mockJoystick)).thenReturn(mockJoystickStatusCommand);
        
        // Create the robot to test
        robot = new Robot();
    }
    
    @Test
    void robotInit_ShouldInitializeJoystickStatusCommand() {
        // Act
        robot.robotInit();
        
        // Assert
        verify(new Joystick(Constants.JOYSTICK_PORT));
        verify(new JoystickStatusCommand(mockJoystick));
        verify(mockJoystickStatusCommand).schedule();
    }
    
    @Test
    void robotPeriodic_WhenPIDTunerHasUpdates_ShouldUpdateDriveSubsystem() {
        // Arrange
        robot.robotInit(); // Initialize the robot
        
        when(mockPidTunerServer.hasUpdates()).thenReturn(true);
        Constants.PIDConfig mockDrivePID = mock(Constants.PIDConfig.class);
        Constants.PIDConfig mockTurnPID = mock(Constants.PIDConfig.class);
        when(mockPidTunerServer.getDrivePID()).thenReturn(mockDrivePID);
        when(mockPidTunerServer.getTurnPID()).thenReturn(mockTurnPID);
        
        // Act
        robot.robotPeriodic();
        
        // Assert
        verify(mockCommandScheduler).run();
        verify(mockPidTunerServer).hasUpdates();
        verify(mockRobotContainer).getDriveSubsystem();
        verify(mockRobotContainer.getDriveSubsystem()).updatePIDValues(mockDrivePID, mockTurnPID);
    }
    
    @Test
    void robotPeriodic_WhenPIDTunerHasNoUpdates_ShouldNotUpdateDriveSubsystem() {
        // Arrange
        robot.robotInit(); // Initialize the robot
        
        when(mockPidTunerServer.hasUpdates()).thenReturn(false);
        
        // Act
        robot.robotPeriodic();
        
        // Assert
        verify(mockCommandScheduler).run();
        verify(mockPidTunerServer).hasUpdates();
        verify(mockRobotContainer, never()).getDriveSubsystem();
    }
    
    @Test
    void autonomousInit_ShouldScheduleAutonomousCommand() {
        // Arrange
        robot.robotInit(); // Initialize the robot
        
        Command mockAutoCommand = mock(Command.class);
        when(mockRobotContainer.getAutonomousCommand()).thenReturn(mockAutoCommand);
        
        // Act
        robot.autonomousInit();
        
        // Assert
        verify(mockRobotContainer).getAutonomousCommand();
        verify(mockAutoCommand).schedule();
    }
    
    @Test
    void teleopInit_ShouldCancelAutonomousCommand() {
        // Arrange
        robot.robotInit(); // Initialize the robot
        
        Command mockAutoCommand = mock(Command.class);
        when(mockRobotContainer.getAutonomousCommand()).thenReturn(mockAutoCommand);
        
        // Set up the autonomous command
        robot.autonomousInit();
        
        // Act
        robot.teleopInit();
        
        // Assert
        verify(mockAutoCommand).cancel();
    }
    
    @Test
    void testInit_ShouldCancelAllCommands() {
        // Arrange
        robot.robotInit(); // Initialize the robot
        
        // Act
        robot.testInit();
        
        // Assert
        verify(mockCommandScheduler).cancelAll();
    }
}