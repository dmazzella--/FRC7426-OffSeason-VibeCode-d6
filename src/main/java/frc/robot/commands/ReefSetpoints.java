// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.Wrist;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public class ReefSetpoints extends Command {

    // Declaring Subsystems
    private final Wrist m_wrist;
    private final Elevator m_elevator;
    private final CoralIntake m_coralIntake;
    private final Limelight m_limelight;
    private final Climber m_climber;
    private final ReefState m_ReefState;

    // Declaring Setpoints/Commands
    public static UpdateSetpoints m_L2ScoreSetpoints;
    public static UpdateSetpoints m_L3ScoreSetpoints;
    public static UpdateSetpoints m_L4ScoreSetpoints;
    private static UpdateSetpoints[] m_setpoints;

    public static Command m_CenterLeftReef;
    public static Command m_CenterRightReef;
    private static Command[] m_Centering;

    //Adding Drivetrain
    private SwerveRequest.RobotCentric limelightDrive = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.025).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025) // Add a 4% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position); // Use open-loop control for drive motors
    public static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

    private double limelight_id;

    private Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

    public ReefSetpoints(double id) {
      // Initialize Subsystems
      m_wrist = Wrist.getInstance();
      m_elevator = Elevator.getInstance();
      m_coralIntake = CoralIntake.getInstance();
      m_limelight = Limelight.getInstance();
      m_climber = Climber.getInstance();
      m_ReefState = ReefState.getInstance();

      //Setpoints
      // m_Idle = new UpdateSetpoints(Setpoints.Idle);
      m_L2ScoreSetpoints = new UpdateSetpoints(Setpoints.L2Setpoint);
      m_L3ScoreSetpoints = new UpdateSetpoints(Setpoints.L3Setpoint);
      m_L4ScoreSetpoints = new UpdateSetpoints(Setpoints.L4Setpoint);
      m_setpoints = new UpdateSetpoints[] {m_L2ScoreSetpoints, m_L3ScoreSetpoints, m_L4ScoreSetpoints};

      //Centering Commands
      m_CenterRightReef = drivetrain.applyRequest(() ->
        limelightDrive.withVelocityX(m_limelight.CenterTy(0, 0.25, 8.15) * Limelight.MaxSpeed) 
            .withVelocityY(m_limelight.CenterTx(0, 0.0, 1) * Limelight.MaxSpeed) 
            .withRotationalRate(m_limelight.CenterRotationRate(0, 0, 1.0) * Limelight.MaxSpeed) 
      );

      m_CenterLeftReef = drivetrain.applyRequest(() ->
        limelightDrive.withVelocityX(m_limelight.CenterTy(1, 0.25, 8.15) * Limelight.MaxSpeed) 
            .withVelocityY(m_limelight.CenterTx(1, 0.0, 1) * Limelight.MaxSpeed) 
            .withRotationalRate(m_limelight.CenterRotationRate(1, 0, 1.0) * Limelight.MaxSpeed)
      ); 

      m_Centering = new Command[] {m_CenterRightReef, m_CenterLeftReef};

      limelight_id = id;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Limelight.id_Detected){
    } else {
      //Something to end command
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
