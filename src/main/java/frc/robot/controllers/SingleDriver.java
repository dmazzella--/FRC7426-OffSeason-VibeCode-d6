// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The bindings class is used to keep all DriverStation inputs organized
 * in an effort to keep our inputs easy to read and modify on the fly.
 */
public class SingleDriver {
    protected static CommandXboxController SINGLE_DRIVER_CONTROLLER = new CommandXboxController(OI.DRIVER_CONTROLLER_ID);

    public static class Auto {
        public static Trigger DriveUp = SINGLE_DRIVER_CONTROLLER.povUp(); 
        public static Trigger DriveDown = SINGLE_DRIVER_CONTROLLER.povDown();
        public static Trigger DriveLeft = SINGLE_DRIVER_CONTROLLER.povLeft(); 
        public static Trigger DriveRight = SINGLE_DRIVER_CONTROLLER.povRight();
        public static Trigger HumanPlayer = SINGLE_DRIVER_CONTROLLER.leftTrigger();
    }

    public static class Setpoint {
        public static Trigger Idle = SINGLE_DRIVER_CONTROLLER.a();

        public static Trigger L4Score = SINGLE_DRIVER_CONTROLLER.leftBumper().and(SINGLE_DRIVER_CONTROLLER.y());
        public static Trigger L3Score = SINGLE_DRIVER_CONTROLLER.leftBumper().and(SINGLE_DRIVER_CONTROLLER.b());
        public static Trigger L2Score = SINGLE_DRIVER_CONTROLLER.leftBumper().and(SINGLE_DRIVER_CONTROLLER.x());

        public static Trigger R4Score = SINGLE_DRIVER_CONTROLLER.rightBumper().and(SINGLE_DRIVER_CONTROLLER.y());
        public static Trigger R3Score = SINGLE_DRIVER_CONTROLLER.rightBumper().and(SINGLE_DRIVER_CONTROLLER.b());
        public static Trigger R2Score = SINGLE_DRIVER_CONTROLLER.rightBumper().and(SINGLE_DRIVER_CONTROLLER.x());

    }

    public final class Climber {
        // public static Trigger Out = SINGLE_DRIVER_CONTROLLER.x();
        public static Trigger In = SINGLE_DRIVER_CONTROLLER.y();
        public static Trigger Speed = SINGLE_DRIVER_CONTROLLER.b();
    }

    public final class Drivetrain {
        public static Trigger Reorient = SINGLE_DRIVER_CONTROLLER.button(8);
    }

    public final class Intake {
        public static Trigger CoralBeam = SINGLE_DRIVER_CONTROLLER.button(7);
        public static Trigger CoralOut = SINGLE_DRIVER_CONTROLLER.rightTrigger(); //button 8
        // public static Trigger AlgaeIn = SINGLE_DRIVER_CONTROLLER.leftBumper();
        // public static Trigger AlgaeOut = SINGLE_DRIVER_CONTROLLER.rightBumper(); //button 7
    }
}

