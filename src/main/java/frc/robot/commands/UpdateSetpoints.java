// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.Wrist;

public class UpdateSetpoints extends Command {
  private Wrist m_wrist;
  private Elevator m_elevator;
  private Setpoints m_setpoint;

  protected Timer m_timer = new Timer();
  private final double m_wristDelay;
  private final double m_elevatorDelay;

  public UpdateSetpoints(Setpoints setpoint) {
    this(setpoint, 0.0, 0.0);
  }

  public UpdateSetpoints(Setpoints setpoint, double wristDelay, double elevatorDelay) {
    m_wrist = Wrist.getInstance();
    m_elevator = Elevator.getInstance();

    m_setpoint = setpoint;

    m_wristDelay = wristDelay;
    m_elevatorDelay = elevatorDelay;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_wristDelay <= 0.0 || m_timer.hasElapsed(m_wristDelay)) {
      m_wrist.updateSetpoint(m_setpoint);
    }
    if(m_elevatorDelay <= 0.0 || m_timer.hasElapsed(m_elevatorDelay)) {
      m_elevator.updateSetpoint(m_setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (
      (m_wristDelay <= 0.0 || m_timer.hasElapsed(m_wristDelay))
      && (m_elevatorDelay <= 0.0 || m_timer.hasElapsed(m_elevatorDelay))
    );
  }
}
