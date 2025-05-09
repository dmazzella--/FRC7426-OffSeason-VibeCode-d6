package frc.robot.obd ;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.shared.Logger;
import frc.robot.subsystems.swerve.TunerConstants;

public class TestAndTuneCommands {

    private TestAndTuneCommands() {
        // private constructor to hide the implicit public one
    }

    static double setpointSteer = 0;

    static  double setpointDrive = 10;

    static double setpointSteerError = 0;
    


    public static Command driveToSetpoint(RobotContainer robotContainer) {
        StringBuilder output = new StringBuilder();
        Command trackerCommand = new Command() {
            @Override
            public void execute() {

                for (int i = 0; i < robotContainer.getDrivetrain().getModules().length; i++) {
                    setpointSteerError = setpointSteer - robotContainer.getDrivetrain().getModules()[i].getDriveMotor().getVelocity().getValueAsDouble();
                    setpointDrive -= (robotContainer.getDrivetrain().getModules()[i].getEncoder().getPosition().getValueAsDouble()) / 4;
                    output.append(robotContainer.getDrivetrain().getModules()[i].getSteerMotor().getDescription() + ": " 
                        + robotContainer.getDrivetrain().getModules()[i].getSteerMotor().getVelocity() + "\n");
                }
            }

            @Override
            public boolean isFinished() {
                if (setpointDrive <= 0) {
                    Logger.println(output + "\nOverall Steer Error: " + setpointSteerError + "\nOverall Drive: " + setpointDrive);
                    return true;
                } else {
                    return false;
                } 
            }
        };
        
        return  new ParallelCommandGroup(trackerCommand, robotContainer.getDrivetrain().applyRequest(() ->robotContainer.getDrive().withVelocityX(TunerConstants.MaxSpeed * 0.5)));
    }
    
}
