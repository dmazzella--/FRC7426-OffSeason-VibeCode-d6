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
import frc.robot.subsystems.body.Wrist;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.climber.Climber;
import frc.robot.controllers.Driver;
import frc.robot.controllers.OI;
import frc.robot.controllers.OperatorPanel;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public class DriverBindings extends AbstractOperator {
    
    private CommandSwerveDrivetrain drivetrain;
    private FieldCentric drive;
    private OI oi;
    private RobotCentric limelightDrive;
    private CoralIntake coralIntake;
    private AlgaeIntake algaeIntake;
    private Climber climber;
    private Wrist m_wrist;
    private RunCommand m_SlowDownDrive = new RunCommand(()->TunerConstants.MaxSpeed = 0.4);

    public DriverBindings(CommandSwerveDrivetrain drivetrain, FieldCentric drive, OI oi, RobotCentric limelightDrive, CoralIntake coralIntake, AlgaeIntake algaeIntake, Climber climber, Limelight limelight, Elevator elevator, Wrist wrist) {
        super(coralIntake, algaeIntake, limelight, elevator);
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.oi = oi;
        this.limelightDrive = limelightDrive;
        this.coralIntake = coralIntake;
        this.algaeIntake = algaeIntake;
        this.climber = climber;
        this.limelight = limelight;
        this.m_wrist = wrist;
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

        Driver.Auto.CenterLeft
            .whileTrue(centerLeftReef());

        Driver.Auto.CenterRight
            .whileTrue(centerRightReef());

        Driver.Auto.AlgaeCenter
            .whileTrue(centerAlgae());

        Driver.Setpoint.Idle
            .onTrue(getReset());
         
        Driver.Drivetrain.Reorient
            .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        Driver.Intake.CoralOut
            .whileTrue(coralOutNoBreak())
            .onFalse(coralIntake.stopCoralIntake());
        
        Driver.Intake.AlgaeOut
            .whileTrue(outakeAlgae(m_wrist.m_setpoint))
            .onFalse(stopAlgaeIntake());

        Driver.Auto.DriveUp
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0.0))
        );

        Driver.Auto.DriveDown
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(-0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0.0))
        );

        Driver.Auto.DriveLeft
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(0)
                        .withVelocityY(0.5)
                        .withRotationalRate(0))
        );
        
        Driver.Auto.DriveRight
            .whileTrue(
                drivetrain.applyRequest(() ->
                limelightDrive
                        .withVelocityX(0)
                        .withVelocityY(-0.5)
                        .withRotationalRate(0.0))
        );

        Driver.Climber.Speed
            .onTrue(new InstantCommand(()->TunerConstants.MaxSpeed = 4.74));

        
        Driver.Climber.In
            .onTrue(
                new SequentialCommandGroup(
                    // new WaitUntilCommand(()->climber.getClimbStart()),
                    // Make the small motor coast in order to avoid damage
                    // Run the big motor and finalize climbing
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->climber.encoderPosition() < -157),
                        climber.runClimber(-0.6)
                    ).andThen(new InstantCommand(()->climber.motorStop()))
                )
            );
    }

}