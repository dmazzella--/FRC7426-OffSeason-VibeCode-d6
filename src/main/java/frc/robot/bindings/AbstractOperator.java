package frc.robot.bindings;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateSetpoints;
import frc.robot.controllers.OI;
import frc.robot.shared.Limelight;
import frc.robot.shared.Logger;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public abstract class AbstractOperator {
    protected CoralIntake coralIntake;
    protected AlgaeIntake algaeIntake;
    protected Limelight limelight;
    protected Elevator elevator;

    public static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

    private OI m_OI = OI.getInstance();

    protected SwerveRequest.RobotCentric limelightDrive = new SwerveRequest.RobotCentric()
        .withDeadband(TunerConstants.MaxSpeed * 0.02).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02) // Add a 4% deadband
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.Position); // Use open-loop control for drive motors

    /* Setting up Bindings for necessary control of the swerve drive platform */
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.09).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.09) // Add a 7% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position); // Use open-loop control for drive motors

    public AbstractOperator(CoralIntake coralIntake, AlgaeIntake algaeIntake, Limelight limelight, Elevator elevator) {
        this.coralIntake = coralIntake;
        this.algaeIntake = algaeIntake;
        this.limelight = limelight;
        this.elevator = elevator;
    }

    public Command getReset() {
        return new ParallelCommandGroup(
            new UpdateSetpoints(Setpoints.Idle),
            coralIntake.stopCoralIntake()
            // algaeIntake.stopAlgaeIntake()
        ); 
    }

    public Command getResetHigh() {
        return new ParallelCommandGroup(
            new UpdateSetpoints(Setpoints.IdleHigh),
            coralIntake.stopCoralIntake()
            // algaeIntake.stopAlgaeIntake()        
        ); 
    }

    public Command getResetAlgae() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                algaeIntake.runAlgaeIntake(0.5, 0.5),
                // algaeIntake.runAlgaeIntake(0.6, 0.6).onlyWhile(()->algaeIntake.algaeBrakeTrigger()).andThen(algaeIntake.stopAlgaeIntake()),
                new UpdateSetpoints(Setpoints.AlgaeIdle),
                coralIntake.stopCoralIntake()
            ) 
        ); 
    }

    public Command intakeAlgae() {
        // return new SequentialCommandGroup(
        //     new ParallelDeadlineGroup(
        //         new WaitUntilCommand(()->algaeIntake.algaeBrakeTrigger()),
        //         algaeIntake.runAlgaeIntake(0.75, 0.75)
        //     )
        // );
        return algaeIntake.runAlgaeIntake(0.4, 0.4);
    }

    public Command outakeAlgae(Setpoints currentSetpoint){
        // if (currentSetpoint.equals(Setpoints.Barge)) {
        //     Logger.println("I am at Barge!");
        //     return algaeIntake.runAlgaeIntake(-0.55, -0.55);
        // }
        return algaeIntake.runAlgaeIntake(-0.45, -0.45);
    }

    public Command stopAlgaeIntake(){
        return algaeIntake.stopAlgaeIntake();
    }

    public Command highAlgaeIntake() {
        return new UpdateSetpoints(Setpoints.HighAlgaeSetpoint);
    }

    public Command lowAlgaeIntake() {
        return new UpdateSetpoints(Setpoints.LowAlgaeSetpoint);
    } 

    public Command groundAlgaeIntake() {
        return new UpdateSetpoints(Setpoints.GroundAlgaeSetpoint);
    }

    public Command centerLeftReef() {
        return drivetrain.applyRequest(() ->
            limelightDrive.withVelocityX(limelight.CenterTy(1, 0.1, 14) * Limelight.MaxSpeed) // Drive vertical with TY and TA and PID
                .withVelocityY(limelight.CenterTx(1,0, 0.16) * Limelight.MaxSpeed) // Drive horizontal with TX
                .withRotationalRate(limelight.CenterRotationRate(1, 0, 0.25) * Limelight.MaxSpeed) // Drive counterclockwise with RZ    
        );
    }

    public Command centerRightReef(){
        return drivetrain.applyRequest(() ->
            limelightDrive.withVelocityX((limelight.CenterTy(0, 0.1, 14) * Limelight.MaxSpeed)) // Drive vertical with TY and TA and PID
                .withVelocityY((limelight.CenterTx(0,0, 0.16) * Limelight.MaxSpeed)) // Drive horizontal with TX
                .withRotationalRate((limelight.CenterRotationRate(0, 0, 0.15) * Limelight.MaxSpeed)) // Drive counterclockwise with RZ    
        ); 
    }

    public Command centerRightReefAuto(){
        return drivetrain.applyRequest(() ->
            limelightDrive.withVelocityX((limelight.CenterTy(0, 0.1, 13.8) * Limelight.MaxSpeed * 1.1)) // Drive vertical with TY and TA and PID
                .withVelocityY((limelight.CenterTx(0, 6.5, 0.16) * Limelight.MaxSpeed * 1.2)) // Drive horizontal with TX
                .withRotationalRate((limelight.CenterRotationRate(0, 0, 0.15) * Limelight.MaxSpeed * 1.2)) // Drive counterclockwise with RZ    
        ); 
    }

    public Command centerAlgae(){
        return drivetrain.applyRequest(() ->
        limelightDrive.withVelocityX(limelight.CenterTy(1, 0.1, 17.5) * Limelight.MaxSpeed * 0.75) // Drive forward with negative Y (forward)
            .withVelocityY(limelight.CenterTx(1, -25.8, 0.16) * Limelight.MaxSpeed * 0.75) // Drive left with negative X (left)
            .withRotationalRate(limelight.CenterRotationRate(1, 0, 0.15) * Limelight.MaxSpeed) // Drive counterclockwise with negative X (left)
        ); 
    }

    // public Command centerAlgaeBack(){
    //     return drivetrain.applyRequest(() ->
    //     limelightDrive.withVelocityX(limelight.CenterTy(0, 0.1, 1.2) * Limelight.MaxSpeed * 1.2) // Drive forward with negative Y (forward)
    //         .withVelocityY(limelight.CenterTx(0, 12.5, 0.16) * Limelight.MaxSpeed * 1.2) // Drive left with negative X (left)
    //         .withRotationalRate(limelight.CenterRotationRate(0, 0, 0.25) * Limelight.MaxSpeed * 1.2) // Drive counterclockwise with negative X (left)
    //     ); 
    // }

    public Command centerLeftHumanPlayer(){
        return drivetrain.applyRequest(() ->
            limelightDrive.withVelocityX((limelight.CenterTy(2, 0.1, 2.15) * 1.25)) // Drive forward with negative Y (forward)
                .withVelocityY((limelight.CenterTx(2, 0.0, 0.75) * 1.25)) // Drive left with negative X (left)
                .withRotationalRate((limelight.CenterRotationRate(2, 0.0, 1.0) * 1.25)) // Drive counterclockwise with negative X (left)
        ); 
    }

    public Command centerRightHumanPlayer(){
        return drivetrain.applyRequest(() ->
            limelightDrive.withVelocityX((limelight.CenterTy(3, 0.1, 2.2) * 1.25)) // Drive forward with negative Y (forward)
                .withVelocityY((limelight.CenterTx(3, 0.0, 0.75) * 1.25)) // Drive left with negative X (left)
                .withRotationalRate((limelight.CenterRotationRate(3, 0.0, 1.0) * 1.25)) // Drive counterclockwise with negative X (left)
        ); 
    }

    public Command driveBack(){
        return drivetrain.applyRequest(() ->
                limelightDrive
                    .withVelocityX(-0.4)
                    .withVelocityY(0)
                    .withRotationalRate(0.0));
    }

    public Command rotateRight(){
        return drivetrain.applyRequest(() ->
            limelightDrive.withVelocityX(0) // Drive forward with negative Y (forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(1.5) // Drive counterclockwise with negative X (left)
        ); 
    }

    public Command rotateLeft(){
        return drivetrain.applyRequest(() ->
            limelightDrive.withVelocityX(0) // Drive forward with negative Y (forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(-1.5) // Drive counterclockwise with negative X (left)
        ); 
    }

    public Command coralOut(){
        return new SequentialCommandGroup(
            new WaitCommand(0.35),
            coralIntake.runCoralIntake(0.2, 0.22).until(()->!coralIntake.highBrakeTrigger()).andThen(coralIntake.stopCoralIntake())
        );
    }

    public Command coralOutAuto(){
        return coralIntake.runCoralIntake(0.16, 0.18).until(()->!coralIntake.lowBrakeTrigger()).andThen(new WaitCommand(0.065));
        //Use High brake trigger for comp 
    }

    public Command coralOutNoBreak(){
        return new RunCommand(()->coralIntake.setPercent(0.16 * coralIntake.getShotSpeedBottom(), 0.18 * coralIntake.getShotSpeedTop()));//.until(()->!coralIntake.highBrakeTrigger()).andThen(coralIntake.stopCoralIntake());
    }

    public SequentialCommandGroup coralBeam(){
        return new SequentialCommandGroup(
            coralIntake.runCoralIntake(0.075, 0.075).until(()->!coralIntake.lowBrakeTrigger()).andThen(
                coralIntake.runCoralIntake(-0.25, -0.25).until(()->coralIntake.lowBrakeTrigger()).andThen(
                coralIntake.stopCoralIntake()))
        );
    }

    
}

