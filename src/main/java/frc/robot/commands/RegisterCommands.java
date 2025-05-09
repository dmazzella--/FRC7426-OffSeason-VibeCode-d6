// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.Wrist;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.manipulator.AlgaeIntake;
import frc.robot.subsystems.manipulator.CoralIntake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.shared.Logger;
import frc.robot.bindings.AbstractOperator;

/**
 * Register commands is used to keep pathplanner's named commands organized
 * and separate from the rest of the code. I definitely enjoy the separation
 * of this approach, but major improvements can be made to optimize the commands
 * themselves. 
 */
public class RegisterCommands extends AbstractOperator {

    private Wrist m_Wrist;
    
    public RegisterCommands(Wrist wrist) { 
        super(CoralIntake.getInstance(), AlgaeIntake.getInstance(), Limelight.getInstance(), Elevator.getInstance());
        this.m_Wrist =  wrist;  

    }

    public void register() {

        // Reset the robot (Idle Positions)
        NamedCommands.registerCommand(
            "Reset", 
            new ParallelDeadlineGroup(
                new WaitUntilCommand(()->elevator.isIdle()),
                getReset()
            )
        );

        //Reset Robot with algae
        NamedCommands.registerCommand(
            "Reset Algae", 
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                getResetAlgae()
            )
        );

        // Reset the robot (Idle Positions)
        NamedCommands.registerCommand(
            "Reset Fast", 
            new ParallelDeadlineGroup(
                new WaitUntilCommand(()->elevator.isIdleHigh()),
                getReset(),
                new InstantCommand(()->Limelight.autoStage++)
            )
        );

        // Beginning of Scoring Commands

        // Human Player Scoring Command
        NamedCommands.registerCommand(
            "Utah Left Human Player", 
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(()->Limelight.validateTag(2)),
                    new UpdateSetpoints(Setpoints.HumanPlayerSetpoint),
                    rotateLeft()
                ),
                new ParallelCommandGroup(
					centerLeftHumanPlayer(),
                    coralIntake.runCoralIntake(-0.5, -0.5)
				).until(()->{return coralIntake.lowBrakeTrigger();}).andThen(getResetHigh())
            )
        );

        // Human Player Scoring Command
        NamedCommands.registerCommand(
            "Left Human Player", 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
					centerLeftHumanPlayer(),
					new UpdateSetpoints(Setpoints.HumanPlayerSetpoint),
                    coralIntake.runCoralIntake(-0.5, -0.5)
				).until(()->{return coralIntake.lowBrakeTrigger();}).andThen(getResetHigh())
            )
        );

        // Human Player Scoring Command
        NamedCommands.registerCommand(
            "Right Human Player", 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
					centerRightHumanPlayer(),
					new UpdateSetpoints(Setpoints.HumanPlayerSetpoint),
                    coralIntake.runCoralIntake(-0.5, -0.5)
				).until(()->{return coralIntake.lowBrakeTrigger();}).andThen(getResetHigh())
            )
        );

        // Human Player Scoring Command
        NamedCommands.registerCommand(
            "Utah Right Human Player", 
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(()->Limelight.validateTag(3)),
                    new UpdateSetpoints(Setpoints.HumanPlayerSetpoint),
                    rotateRight()
                ),
                new ParallelCommandGroup(
					centerRightHumanPlayer(),
                    coralIntake.runCoralIntake(-0.5, -0.5)
				).until(()->{return coralIntake.lowBrakeTrigger();}).andThen(getResetHigh())
            )
        );

        // Scoring on the 4th Level on the Left Side
        NamedCommands.registerCommand(
            "Left L4",
				new ParallelCommandGroup(
					centerLeftReef().until(()->!coralIntake.highBrakeTrigger()),
                    new SequentialCommandGroup(
                        coralBeam(),
                        new ParallelDeadlineGroup(
                            new WaitCommand(0.1),
                            coralIntake.runCoralIntake(-0.5, -0.5)
                        ),
                        coralIntake.stopCoralIntake()
                    ),
					new SequentialCommandGroup(
							// new WaitUntilCommand(()->limelight.isCloseLeft()),
						new UpdateSetpoints(Setpoints.L4Setpoint),
						new WaitUntilCommand(()->elevator.isL4()),
                        new WaitCommand(0.65),
						coralOutAuto() 
					)
				)
        );

        // Scoring on the 4th Level on the Left Side
        NamedCommands.registerCommand(
            "Utah Left L4",
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->(Limelight.validateTag(1))), 
                        rotateLeft()
                    ),
                    new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                            new WaitCommand(2),
                            centerLeftReef()
                        ),
                        new SequentialCommandGroup(
                            coralBeam(),
                            new ParallelDeadlineGroup(
                                new WaitCommand(0.1),
                                coralIntake.runCoralIntake(-0.5, -0.5)
                            ),
                            coralIntake.stopCoralIntake()
                        ),
                        new SequentialCommandGroup(
                                // new WaitUntilCommand(()->limelight.isCloseLeft()),
                            new UpdateSetpoints(Setpoints.L4Setpoint),
                            new WaitUntilCommand(()->elevator.isL4()),
                            new WaitCommand(1),
                            coralOutAuto()
                        )
                    )   
                )
        );

        // Scoring on the 4th Level on the Right Side
        NamedCommands.registerCommand(
            "Right L4",
            new ParallelCommandGroup(
                centerRightReef().until(()->!coralIntake.highBrakeTrigger()),
                new SequentialCommandGroup(
                    coralBeam(),
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.1),
                        coralIntake.runCoralIntake(-0.5, -0.5)
                    ),
                    coralIntake.stopCoralIntake()
                ),
                new SequentialCommandGroup(
                    // new WaitUntilCommand(()->limelight.isCloseLeft()),
                    new UpdateSetpoints(Setpoints.L4Setpoint),
                    new WaitUntilCommand(()->elevator.isL4()),
                    new WaitCommand(0.75),
                    coralOutAuto()
                )
            )
        );
                
        // Scoring on the 4th Level on the Right Side
        NamedCommands.registerCommand(
            "Utah Right L4",
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->(Limelight.validateTag(0))),
                        rotateRight()
                    ),
                    new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                            new WaitCommand(2),
                            centerRightReef()
                        ),
                        new SequentialCommandGroup(
                            coralBeam(),
                            new ParallelDeadlineGroup(
                                new WaitCommand(0.1),
                                coralIntake.runCoralIntake(-0.5, -0.5)
                            ),
                            coralIntake.stopCoralIntake()
                        ),
                        new SequentialCommandGroup(
                                // new WaitUntilCommand(()->limelight.isCloseLeft()),
                            new UpdateSetpoints(Setpoints.L4Setpoint),
                            new WaitUntilCommand(()->elevator.isL4()),
                            new WaitCommand(1),
                            coralOutAuto()
                        )
                    )   
                )
        );

        //VEGAS COMMANDS FOR L4 ARE HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

        NamedCommands.registerCommand(
            "Vegas Right L4",
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->(Limelight.validateTag(0)))
                        // rotateRight()
                    ),
                    new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                            new WaitCommand(1.5),
                            centerRightReef()
                        ),
                        new SequentialCommandGroup(
                            coralBeam(),
                            new ParallelDeadlineGroup(
                                new WaitCommand(0.1),
                                coralIntake.runCoralIntake(-0.5, -0.5)
                            ),
                            coralIntake.stopCoralIntake()
                        ),
                        new SequentialCommandGroup(
                            new UpdateSetpoints(Setpoints.L4Setpoint),
                            new WaitUntilCommand(()->elevator.isL4()),
                            new WaitCommand(0.25),
                            coralOutAuto()
                        )
                    )   
                )
        );
    
        NamedCommands.registerCommand(
            "Vegas Left L4",
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->(Limelight.validateTag(1)))
                        // rotateLeft()
                    ),
                    new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                            new WaitCommand(1.5),
                            centerLeftReef()
                        ),
                        new SequentialCommandGroup(
                            coralBeam(),
                            new ParallelDeadlineGroup(
                                new WaitCommand(0.1),
                                coralIntake.runCoralIntake(-0.5, -0.5)
                            ),
                            coralIntake.stopCoralIntake()
                        ),
                        new SequentialCommandGroup(
                            new UpdateSetpoints(Setpoints.L4Setpoint),
                            new WaitUntilCommand(()->elevator.isL4()),
                            new WaitCommand(0.25),
                            coralOutAuto()
                        )
                    )   
                )
        );

        //Alage center 
        NamedCommands.registerCommand(
            "Center Algae Low",
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(1.5),
                    intakeAlgae(),
                    new UpdateSetpoints(Setpoints.LowAlgaeSetpoint),
                    new SequentialCommandGroup(
                        new WaitCommand(0.4), 
                        centerAlgae()
                    )
                ),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5),
                    intakeAlgae(),
                    driveBack()
                )
            )
        );

        NamedCommands.registerCommand(
            "Center Algae High",
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(1.8),
                    intakeAlgae(),
                    new UpdateSetpoints(Setpoints.HighAlgaeSetpoint),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5), 
                        centerAlgae()
                    )
                ),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.75),
                    intakeAlgae(),
                    driveBack()
                )
            )
        );

        NamedCommands.registerCommand(
            "Barge Back",
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(1.1),
                    driveBack(),
                    new UpdateSetpoints(Setpoints.BargeBack, 0.85, 0),
                    new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                            new WaitCommand(0.9),
                            algaeIntake.runAlgaeIntake(0.5, 0.5)
                        ),
                        outakeAlgae(m_Wrist.m_setpoint)
                    )
                ),
                stopAlgaeIntake()
            )
        );
                
        // Pipeline Commands
        NamedCommands.registerCommand("Pipeline Center",  
            new InstantCommand(() -> Limelight.autoStage = -5)
        );

        // Alage Commands>
        NamedCommands.registerCommand(
            "Algae High",
            new SequentialCommandGroup(
                new UpdateSetpoints(Setpoints.HighAlgaeSetpoint),
                intakeAlgae()
            )
        );

        NamedCommands.registerCommand(
        "Idle High", 
            getResetHigh());
    }

}
