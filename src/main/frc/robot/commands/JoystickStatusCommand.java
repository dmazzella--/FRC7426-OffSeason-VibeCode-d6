package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to monitor joystick connection status
 */
public class JoystickStatusCommand extends Command {
    private final Joystick joystick;
    private boolean wasConnected = false;
    
    /**
     * Create a command to monitor joystick connection status
     * 
     * @param joystick The joystick to monitor
     */
    public JoystickStatusCommand(Joystick joystick) {
        this.joystick = joystick;
    }
    
    @Override
    public void initialize() {
        wasConnected = joystick.isConnected();
        updateStatus();
    }
    
    @Override
    public void execute() {
        boolean isConnected = joystick.isConnected();
        
        // Only update if the connection status has changed
        if (isConnected != wasConnected) {
            wasConnected = isConnected;
            updateStatus();
        }
    }
    
    /**
     * Update the joystick status on SmartDashboard
     */
    private void updateStatus() {
        SmartDashboard.putBoolean("Joystick Connected", wasConnected);
        
        if (wasConnected) {
            SmartDashboard.putString("Joystick Status", "Connected");
            SmartDashboard.putString("Joystick Name", joystick.getName());
            SmartDashboard.putNumber("Joystick Button Count", joystick.getButtonCount());
        } else {
            SmartDashboard.putString("Joystick Status", "Disconnected");
        }
    }
    
    @Override
    public boolean isFinished() {
        // This command never finishes on its own
        return false;
    }
}