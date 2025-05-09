package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
  private final SwerveDriveSubsystem driveSubsystem;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotationSupplier;
  
  /**
   * Create a new DefaultDriveCommand
   * 
   * @param driveSubsystem The drive subsystem
   * @param xSupplier Forward/backward speed supplier
   * @param ySupplier Left/right speed supplier
   * @param rotationSupplier Rotation speed supplier
   */
  public DefaultDriveCommand(
    SwerveDriveSubsystem driveSubsystem,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier rotationSupplier
  ) {
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    
    addRequirements(driveSubsystem);
  }
  
  @Override
  public void execute() {
    driveSubsystem.drive(
      xSupplier.getAsDouble(),
      ySupplier.getAsDouble(),
      rotationSupplier.getAsDouble(),
      true // Field-relative by default
    );
  }
  
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}