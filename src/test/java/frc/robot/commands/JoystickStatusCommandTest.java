package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class JoystickStatusCommandTest {
    // Mocks
    private Joystick mockJoystick;
    
    // System under test
    private JoystickStatusCommand command;
    
    @BeforeEach
    void setUp() {
        // Arrange - Create mocks
        mockJoystick = mock(Joystick.class);
        
        // Mock static methods
        mockStatic(SmartDashboard.class);
        
        // Create the command to test
        command = new JoystickStatusCommand(mockJoystick);
    }
    
    @Test
    void initialize_WhenJoystickConnected_ShouldUpdateSmartDashboard() {
        // Arrange
        when(mockJoystick.isConnected()).thenReturn(true);
        when(mockJoystick.getName()).thenReturn("Test Joystick");
        when(mockJoystick.getButtonCount()).thenReturn(32);
        
        // Act
        command.initialize();
        
        // Assert
        verify(SmartDashboard.class).putBoolean("Joystick Connected", true);
        verify(SmartDashboard.class).putString("Joystick Status", "Connected");
        verify(SmartDashboard.class).putString("Joystick Name", "Test Joystick");
        verify(SmartDashboard.class).putNumber("Joystick Button Count", 32);
    }
    
    @Test
    void initialize_WhenJoystickDisconnected_ShouldUpdateSmartDashboard() {
        // Arrange
        when(mockJoystick.isConnected()).thenReturn(false);
        
        // Act
        command.initialize();
        
        // Assert
        verify(SmartDashboard.class).putBoolean("Joystick Connected", false);
        verify(SmartDashboard.class).putString("Joystick Status", "Disconnected");
        verify(mockJoystick, never()).getName();
        verify(mockJoystick, never()).getButtonCount();
    }
    
    @Test
    void execute_WhenConnectionStatusChanges_ShouldUpdateSmartDashboard() {
        // Arrange
        when(mockJoystick.isConnected()).thenReturn(false);
        command.initialize();
        
        // Reset mocks to verify only the new calls
        reset(SmartDashboard.class);
        
        // Change connection status
        when(mockJoystick.isConnected()).thenReturn(true);
        when(mockJoystick.getName()).thenReturn("Test Joystick");
        when(mockJoystick.getButtonCount()).thenReturn(32);
        
        // Act
        command.execute();
        
        // Assert
        verify(SmartDashboard.class).putBoolean("Joystick Connected", true);
        verify(SmartDashboard.class).putString("Joystick Status", "Connected");
        verify(SmartDashboard.class).putString("Joystick Name", "Test Joystick");
        verify(SmartDashboard.class).putNumber("Joystick Button Count", 32);
    }
    
    @Test
    void execute_WhenConnectionStatusDoesNotChange_ShouldNotUpdateSmartDashboard() {
        // Arrange
        when(mockJoystick.isConnected()).thenReturn(true);
        when(mockJoystick.getName()).thenReturn("Test Joystick");
        when(mockJoystick.getButtonCount()).thenReturn(32);
        command.initialize();
        
        // Reset mocks to verify no new calls
        reset(SmartDashboard.class);
        
        // Act
        command.execute();
        
        // Assert
        verify(SmartDashboard.class, never()).putBoolean(anyString(), anyBoolean());
        verify(SmartDashboard.class, never()).putString(anyString(), anyString());
        verify(SmartDashboard.class, never()).putNumber(anyString(), anyDouble());
    }
    
    @Test
    void isFinished_ShouldAlwaysReturnFalse() {
        // Act
        boolean result = command.isFinished();
        
        // Assert
        assertFalse(result);
    }
}