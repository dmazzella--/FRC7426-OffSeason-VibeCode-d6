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
public class Driver {
    protected static CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(OI.DRIVER_CONTROLLER_ID);

    public static class Auto {
        public static Trigger DriveUp = DRIVER_CONTROLLER.povUp(); 
        public static Trigger DriveDown = DRIVER_CONTROLLER.povDown();
        public static Trigger DriveLeft = DRIVER_CONTROLLER.povLeft(); 
        public static Trigger DriveRight = DRIVER_CONTROLLER.povRight();
        public static Trigger CenterLeft = DRIVER_CONTROLLER.leftBumper();
        public static Trigger CenterRight = DRIVER_CONTROLLER.rightBumper();
        public static Trigger AlgaeCenter = DRIVER_CONTROLLER.x();
    }

    public static class Setpoint {
        // public static Trigger L1Setpoint = new Trigger(()->false); //Add
        public static Trigger Idle = DRIVER_CONTROLLER.a();
    }

    public final class Climber {
        public static Trigger In = DRIVER_CONTROLLER.y();
        public static Trigger Speed = DRIVER_CONTROLLER.b();
    }

    public final class Drivetrain {
        public static Trigger Reorient = DRIVER_CONTROLLER.button(8);
    }

    public final class Intake {
        public static Trigger CoralBeam = DRIVER_CONTROLLER.button(7);
        public static Trigger CoralOut = DRIVER_CONTROLLER.rightTrigger(); //button 8
        // public static Trigger AlgaeIn = DRIVER_CONTROLLER.leftTrigger();
        public static Trigger AlgaeOut = DRIVER_CONTROLLER.leftTrigger();
    }
}

