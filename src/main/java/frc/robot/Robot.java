package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.JoystickStatusCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.web.PIDTunerServer;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private PIDTunerServer pidTunerServer;
  private Command joystickStatusCommand;

  @Override
  public void robotInit() {
    // Initialize robot container
    robotContainer = new RobotContainer();
    
    // Start PID tuner web server on port 8081
    pidTunerServer = new PIDTunerServer(8081);
    pidTunerServer.start();
    
    System.out.println("PID Tuner available at http://roborio-team-frc.local:8081");
    
    // Initialize PathPlanner logging
    PathPlannerLogging.setLogActivePathCallback((path) -> {
      // This would typically visualize the active path on a Field2d
    });
    
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // This would typically visualize the target pose on a Field2d
    });
    
    // Start joystick status monitoring
    joystickStatusCommand = new JoystickStatusCommand(new edu.wpi.first.wpilibj.Joystick(Constants.JOYSTICK_PORT));
    joystickStatusCommand.schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Update PID values if they've been changed through the web interface
    if (pidTunerServer.hasUpdates()) {
      robotContainer.getDriveSubsystem().updatePIDValues(
        pidTunerServer.getDrivePID(),
        pidTunerServer.getTurnPID()
      );
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}