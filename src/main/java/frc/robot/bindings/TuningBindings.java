package frc.robot.bindings;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.controllers.OI;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public class TuningBindings extends AbstractOperator {

    private OI oi ;
    private CommandSwerveDrivetrain drivetrain;
    private FieldCentric drive;

    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
 
    public TuningBindings(OI oi, CommandSwerveDrivetrain drivetrain, FieldCentric drive) {
        super(CoralIntake.getInstance(), AlgaeIntake.getInstance(), Limelight.getInstance(), Elevator.getInstance());
        this.oi = oi;   
        this.drivetrain = drivetrain;
        this.drive = drive;
    }

    public void setBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-oi.getDriver().getLeftY() * TunerConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-oi.getDriver().getLeftX() * TunerConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-oi.getDriver().getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); 
        
        oi.getDriver().button(8).onTrue(getReset());

        oi.getDriver().button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
               
        // oi.getDriver().b().whileTrue(drivetrain.applyRequest(() ->
        //      point.withModuleDirection(new Rotation2d(-oi.getDriver().getLeftY(), -oi.getDriver().getLeftX()))
        //  ));

        // DriverBindings.Drivetrain.ForwardStraight.whileTrue(drivetrain.applyRequest(() ->
        //      forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // Bindings.Drivetrain.BackwardsStraight.whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );


        // Step 1: Tune drivetrain with SysId commands (Done)
        // Step 2: Implement a way to keep the claw (wrist) at a 0 position (could use setpoint mode)
        // Step 3: Determine elevator behavior
        // Run SysId routines when holding Y/A/B/X.
        // Note that each routine should be run exactly once in a single log.
        oi.getDriver().y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        oi.getDriver().b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        oi.getDriver().a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        oi.getDriver().x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // Temporary commands to start and stop Signal Logger - Used for System Identification
        oi.getDriver().leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        oi.getDriver().rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    }
}
