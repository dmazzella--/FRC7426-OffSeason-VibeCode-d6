package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.UpdateSetpoints;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.controllers.OperatorPanel;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;

public class OperatorPanelBindings extends AbstractOperator {

	public OperatorPanelBindings(CoralIntake coralIntake, AlgaeIntake algaeIntake, Limelight limelight, Elevator elevator) {
		super(coralIntake, algaeIntake, limelight, elevator);
	}
    
	public void setBindings() {
		intakeBindings();
		scoreBindings();
	}

	public void intakeBindings(){
		OperatorPanel.Intake.HumanPlayerOn
            .whileTrue(
				new ParallelCommandGroup(
					new UpdateSetpoints(Setpoints.HumanPlayerSetpoint),
                    coralIntake.runCoralIntake(-0.5, -0.5)
				).until(()->{return coralIntake.lowBrakeTrigger();}).andThen(getReset())
			);

		OperatorPanel.Intake.HumanPlayerOff
            .onTrue(getReset());

		OperatorPanel.Intake.AlgaeIntakeHigh
            .whileTrue(
			new ParallelCommandGroup(
				new UpdateSetpoints(Setpoints.HighAlgaeSetpoint)
				// centerAlgaeIntake()
			))
            .onFalse(getReset());
        
        OperatorPanel.Intake.AlgaeIntakeLow
            .whileTrue(new UpdateSetpoints(Setpoints.LowAlgaeSetpoint))
            .onFalse(getReset());
		
		OperatorPanel.Intake.AlgaeIntakeGround 
			.whileTrue(new UpdateSetpoints(Setpoints.GroundAlgaeSetpoint))
			.onFalse(getReset());
	}

	public void scoreBindings() {
		OperatorPanel.Score.L4Score
			.whileTrue(new UpdateSetpoints(Setpoints.L4Setpoint))
			.onFalse(getReset());
		
		OperatorPanel.Score.L3Score
			.whileTrue(new UpdateSetpoints(Setpoints.L3Setpoint))
			.onFalse(getReset());
		
		OperatorPanel.Score.L2Score
			.whileTrue(new UpdateSetpoints(Setpoints.L2Setpoint))
			.onFalse(getReset());

		OperatorPanel.Score.AlgaeProcessor
            .whileTrue(new UpdateSetpoints(Setpoints.Processor))
            .onFalse(getReset());

        OperatorPanel.Score.AlgaeBarge
            .whileTrue(new UpdateSetpoints(Setpoints.Idle))
            .onFalse(getReset());   
	}
}
