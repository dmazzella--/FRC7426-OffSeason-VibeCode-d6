// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

public class ManipulatorConstants {
    public static final AlgaeSetup kAlgaeIntakeConfig = new AlgaeSetup(21, 22);

    public static final CoralSetup kCoralIntakeConfig = new CoralSetup(23, 24);

    public record AlgaeSetup(int leftId, int rightId) {};
    public record CoralSetup(int topId, int bottomId) {};
}
