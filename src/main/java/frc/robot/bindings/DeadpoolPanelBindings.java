package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.UpdateSetpoints;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.Wrist;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.controllers.DeadpoolPanel;
import frc.robot.controllers.OperatorPanel;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;
import frc.robot.subsystems.swerve.TunerConstants;

public class DeadpoolPanelBindings extends AbstractOperator {

    private Climber climber;
	private Wrist m_wrist;

	public DeadpoolPanelBindings(CoralIntake coralIntake, AlgaeIntake algaeIntake, Limelight limelight, Elevator elevator, Climber climber, Wrist wrist) {
		super(coralIntake, algaeIntake, limelight, elevator);
		this.climber = climber;
		this.m_wrist = wrist;
	}
    
	public void setBindings() {
		intakeBindings();
		scoreBindings();
	}

	public void intakeBindings(){
		DeadpoolPanel.Intake.HumanPlayerOn
            .whileTrue(
				new ParallelCommandGroup(
					new UpdateSetpoints(Setpoints.HumanPlayerSetpoint),
                    coralIntake.runCoralIntake(-0.5, -0.5)
				)
				// ).until(()->{return coralIntake.lowBrakeTrigger();}).andThen(getReset())
			);

		DeadpoolPanel.Intake.HumanPlayerOff
            .onTrue(getReset());

		DeadpoolPanel.Intake.AlgaeIntakeHigh
            .whileTrue(
				new ParallelCommandGroup(
					new UpdateSetpoints(Setpoints.HighAlgaeSetpoint),
					intakeAlgae()
				)
			)
            .onFalse(getResetAlgae());
        
        DeadpoolPanel.Intake.AlgaeIntakeLow
			.whileTrue(
				new ParallelCommandGroup(
					new UpdateSetpoints(Setpoints.LowAlgaeSetpoint),
					intakeAlgae()
				)
			)
            .onFalse(getResetAlgae());
		
		DeadpoolPanel.Intake.AlgaeIntakeGround 
			.whileTrue(
				new ParallelCommandGroup(
					new UpdateSetpoints(Setpoints.GroundAlgaeSetpoint),
					intakeAlgae()
				)
			)
			.onFalse(getResetAlgae());

		DeadpoolPanel.Intake.CoralBeam
            .onTrue(coralBeam());

		DeadpoolPanel.Climber.Out
            .onTrue(
                new SequentialCommandGroup(
                    // Bring upper body up and allowing big motor to spin
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->elevator.isClimbHigh()),
                        new UpdateSetpoints(Setpoints.ClimberStart), 
                        climber.runClimber(-1).onlyWhile(()->climber.encoderPosition() < -97)
                    ),

                    // // A little delay before spinning the small motor while still running the big motor
                    new ParallelDeadlineGroup(new WaitCommand(0.25), climber.runClimber(-1).onlyWhile(()->climber.encoderPosition() < -97)),

                    // Run the small and big motor at the same time
                    new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                            new WaitUntilCommand(()->climber.encoderPosition() < -97),
                            climber.runClimber(-1)
                        ).andThen(new InstantCommand(()->climber.motorStop())),
                        new ParallelDeadlineGroup(
                            new WaitUntilCommand(()->climber.climbBrakeTrigger())
						)
                    ),

                    // Bring the upper body down and hold the small motor in place
                    new UpdateSetpoints(Setpoints.ClimberEnd),
					new ParallelCommandGroup(
						new RunCommand(()->TunerConstants.MaxSpeed = 0.75),
						new RunCommand(()->TunerConstants.MaxAngularRate = 0.75)
					)
					// new InstantCommand(()->climber.startClimber())

                )
            );


	}

	public void scoreBindings() {
		DeadpoolPanel.Score.L4Score
			.whileTrue(new UpdateSetpoints(Setpoints.L4Setpoint))
			.onFalse(getReset());
		
		DeadpoolPanel.Score.L3Score
			.whileTrue(new UpdateSetpoints(Setpoints.L3Setpoint))
			.onFalse(getReset());
		
		DeadpoolPanel.Score.L2Score
			.whileTrue(new UpdateSetpoints(Setpoints.L2Setpoint))
			.onFalse(getReset());
		
		DeadpoolPanel.Score.L1Score
			.whileTrue(new UpdateSetpoints(Setpoints.L1Setpoint))
			.onFalse(getReset());

		DeadpoolPanel.Score.R4Score
			.whileTrue(
				new ParallelCommandGroup(
					new UpdateSetpoints(Setpoints.BargeBack, 0.85, 0),
					new SequentialCommandGroup(
						new ParallelDeadlineGroup(
							new WaitCommand(0.9),
							algaeIntake.runAlgaeIntake(0.5, 0.5)
						),
						outakeAlgae(m_wrist.m_setpoint)
					)
				)
			)
			.onFalse(getReset());
		
		DeadpoolPanel.Score.R3Score
			.whileTrue(new UpdateSetpoints(Setpoints.L3Setpoint))
			.onFalse(getReset());
		
		DeadpoolPanel.Score.R2Score
			.whileTrue(new UpdateSetpoints(Setpoints.L2Setpoint))
			.onFalse(getReset());
		
		DeadpoolPanel.Score.R1Score
			.whileTrue(new UpdateSetpoints(Setpoints.L1Setpoint))
			.onFalse(getReset());

		DeadpoolPanel.Score.Idle 
			.onTrue(getReset());
		
		DeadpoolPanel.Score.AlgaeProcessor
            .whileTrue(new UpdateSetpoints(Setpoints.Processor))
            .onFalse(getReset());

        DeadpoolPanel.Score.AlgaeBarge
            .whileTrue(new UpdateSetpoints(Setpoints.Barge))
            .onFalse(getReset());   
	}
}
