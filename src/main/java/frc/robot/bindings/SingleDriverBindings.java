package frc.robot.bindings;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.UpdateSetpoints;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.climber.Climber;
import frc.robot.controllers.OI;
import frc.robot.controllers.OperatorPanel;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.controllers.SingleDriver;

public class SingleDriverBindings extends AbstractOperator {
    
    private CommandSwerveDrivetrain drivetrain;
    private FieldCentric drive;
    private OI oi;
    private RobotCentric limelightDrive;
    private CoralIntake coralIntake;
    private AlgaeIntake algaeIntake;
    private Climber climber;
    private RunCommand m_SlowDownDrive = new RunCommand(()->TunerConstants.MaxSpeed = 0.4);

    public SingleDriverBindings(CommandSwerveDrivetrain drivetrain, FieldCentric drive, OI oi, RobotCentric limelightDrive, CoralIntake coralIntake, AlgaeIntake algaeIntake, Climber climber, Limelight limelight, Elevator elevator) {
        super(coralIntake, algaeIntake, limelight, elevator);
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.oi = oi;
        this.limelightDrive = limelightDrive;
        this.coralIntake = coralIntake;
        this.algaeIntake = algaeIntake;
        this.climber = climber;
        this.limelight = limelight;
    }

    public void setBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-oi.getDriver().getLeftY() * TunerConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-oi.getDriver().getLeftX() * TunerConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-oi.getDriver().getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );  

        SingleDriver.Setpoint.Idle
            .onTrue(getReset());

        SingleDriver.Drivetrain.Reorient
            .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        SingleDriver.Intake.CoralOut
            .whileTrue(coralOutNoBreak());

        // SingleDriver.Intake.AlgaeIn
        //     .whileTrue(algaeIntake.runAlgaeIntake(0.3, 0.3))
        //     .onFalse(algaeIntake.runAlgaeIntake(0.0, 0.0));
        
        // SingleDriver.Intake.AlgaeOut
        //     .whileTrue(algaeIntake.runAlgaeIntake(-0.3, -0.3))
        //     .onFalse(algaeIntake.runAlgaeIntake(0.0, 0.0));

        SingleDriver.Auto.DriveUp
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0.0))
        );

        SingleDriver.Auto.DriveDown
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(-0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0.0))
        );

        SingleDriver.Auto.DriveLeft
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(0)
                        .withVelocityY(0.5)
                        .withRotationalRate(0))
        );
        
        SingleDriver.Auto.DriveRight
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(0)
                        .withVelocityY(-0.5)
                        .withRotationalRate(0.0))
        );

        SingleDriver.Climber.Speed
            .onTrue(new InstantCommand(()->TunerConstants.MaxSpeed = 4.74));

        
        SingleDriver.Climber.In
            .onTrue(
                new SequentialCommandGroup(
                    // Make the small motor coast in order to avoid damage
                    // Run the big motor and finalize climbing
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->climber.encoderPosition() < -167),
                        climber.runClimber(-0.6)
                    ).andThen(new InstantCommand(()->climber.motorStop()))
                )
            );
               
        scoreBindings();
    }

    public void scoreBindings() {

		SingleDriver.Setpoint.L4Score
			.whileTrue(
				new ParallelCommandGroup(
						centerLeftReef(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()->limelight.isCloseLeft()),
							new UpdateSetpoints(Setpoints.L4Setpoint),
							new WaitUntilCommand(()->elevator.isL4()),
							coralOut() 
						)
				))
			.onFalse(getReset());

        SingleDriver.Setpoint.L3Score
			.whileTrue(
				new ParallelCommandGroup(
						centerLeftReef(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()->limelight.isCloseLeft()),
							new UpdateSetpoints(Setpoints.L3Setpoint),
							new WaitUntilCommand(()->elevator.isL3()),
							coralOut() 
						)
				))
			.onFalse(getReset());

        SingleDriver.Setpoint.L2Score
            .whileTrue(
				new ParallelCommandGroup(
						centerLeftReef(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()->limelight.isCloseLeft()),
							new UpdateSetpoints(Setpoints.L2Setpoint),
							new WaitUntilCommand(()->elevator.isL2()),
							coralOut() 
						)
				))
            .onFalse(getReset());

        SingleDriver.Setpoint.R4Score
			.whileTrue(
				new ParallelCommandGroup(
						centerRightReef(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()->limelight.isCloseRight()),
							new UpdateSetpoints(Setpoints.L4Setpoint),
							new WaitUntilCommand(()->elevator.isL4()),
							coralOut() 
						)
				))             
			.onFalse(getReset());

        SingleDriver.Setpoint.R3Score
			.whileTrue(
				new ParallelCommandGroup(
						centerRightReef(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()->limelight.isCloseRight()),
							new UpdateSetpoints(Setpoints.L3Setpoint),
							new WaitUntilCommand(()->elevator.isL3()),
							coralOut() 
						)
				))
			.onFalse(getReset());

        SingleDriver.Setpoint.R2Score
            .whileTrue(
				new ParallelCommandGroup(
						centerRightReef(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()->limelight.isCloseRight()),
							new UpdateSetpoints(Setpoints.L2Setpoint),
							new WaitUntilCommand(()->elevator.isL2()),
							coralOut() 
						)
				))
            .onFalse(getReset());
	}
}
