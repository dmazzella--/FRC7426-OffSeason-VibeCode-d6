package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RegisterCommands;
import frc.robot.commands.UpdateSetpoints;
import frc.robot.controllers.OperatorPanel;
import frc.robot.controllers.XBoxOperator;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;

public class OperatorXBoxBindings extends AbstractOperator{

    public OperatorXBoxBindings(CoralIntake coralIntake, AlgaeIntake algaeIntake, Limelight limelight, Elevator elevator) {
		super(coralIntake, algaeIntake, limelight, elevator);
	}
    
	public void setBindings() {
		// intakeBindings();
		scoreBindings();
	}

	public void intakeBindings(){
        XBoxOperator.Intake.CoralBeam
            .whileTrue(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.1), 
                        coralIntake.runCoralIntake(-0.1, -0.1)),
                    coralIntake.runCoralIntake(0.8, 0.8)
                )
            )
            .onFalse(coralIntake.runCoralIntake(0, 0));

	// 	XBoxOperator.Intake.AlgaeIntakeHigh
    //         .whileTrue(registerCommands.m_HighAlgaeIntake)
    //         .onFalse(getReset());
        
    //     XBoxOperator.Intake.AlgaeIntakeLow
    //         .whileTrue(registerCommands.m_LowAlgaeIntake)
    //         .onFalse(getReset());
		
	// 	XBoxOperator.Intake.AlgaeIntakeGround 
	// 		.whileTrue(registerCommands.m_GroundAlgaeIntake)
	// 		.onFalse(getReset());
	}

	public void scoreBindings() {
		XBoxOperator.Setpoint.L4Setpoint
			.onTrue(new UpdateSetpoints(Setpoints.L4Setpoint))
			.onFalse(getReset());

        XBoxOperator.Setpoint.L3Setpoint
			.onTrue(new UpdateSetpoints(Setpoints.L3Setpoint))
			.onFalse(getReset());

		XBoxOperator.Setpoint.L2Setpoint
            .onTrue(new UpdateSetpoints(Setpoints.L2Setpoint))
            .onFalse(getReset());

		XBoxOperator.Setpoint.L1Setpoint
			.onTrue(new UpdateSetpoints(Setpoints.Idle))
			.onFalse(getReset());

		// XBoxOperator.Setpoint.AlgaeProcessor
        //     .whileTrue(new UpdateSetpoints(Setpoints.Processor))
        //     .onFalse(getReset());

        // XBoxOperator.Setpoint.AlgaeBarge
        //     .whileTrue(new UpdateSetpoints(Setpoints.Idle))
        //     .onFalse(getReset());   
	}
}
