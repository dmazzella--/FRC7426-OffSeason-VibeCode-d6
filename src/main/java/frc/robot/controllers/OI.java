// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.shared.Alert;

/**
 * The Operator Interface singleton is where the robot's input
 * methods are created, being two Xbox controllers for both
 * operating and driving the robot.
 */
public class OI {
    private static OI INSTANCE;

    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;

    public enum ControllerType {
        OPERATOR_PANEL,
        POSEIDON_PANEL,
        XBOX_CONTROLLER
    }

    public static OI getInstance(ControllerType controllerType) {
        if(INSTANCE == null) {
            INSTANCE = new OI(controllerType);

        }
        return INSTANCE;
    }

    public static OI getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new OI(ControllerType.XBOX_CONTROLLER);
        }
        return INSTANCE;
    }

    // Declaring controller objects
    private static CommandXboxController m_operatorXBox;
    private static CommandJoystick m_operatorUSB;

    // Declaring Alert objects
    private Alert m_alertDriver;
    private Alert m_alertOperator;

    // Format used to alert when a controller is disconnected
    private String m_alertFormat = "The %s controller is not connected. (Port %d)";

    private OI(ControllerType controllerType) {
        if(controllerType == ControllerType.OPERATOR_PANEL) {
            m_operatorUSB = OperatorPanel.m_operatorUSB;
        } else if(controllerType == ControllerType.POSEIDON_PANEL) {
            // TODO: Implement Poseidon Panel
        } else {
            m_operatorXBox = XBoxOperator.m_operator;
        }  
    }

    /**
     * @return The driver controller input.
     */
    public CommandXboxController getDriver() {
        return SingleDriver.SINGLE_DRIVER_CONTROLLER;
    }

    /**
     * @return The operator controller input.
     */
    public CommandXboxController getOperatorXbox() {
        return m_operatorXBox;
    }

    /**
     * @return The operator controller input.
     */
    public CommandJoystick getOperatorUSB() {
        return m_operatorUSB;
    }

    /**
     * Checks if the driver and or operator controller are connected
     * and are the correct controller types. Will show an alert if a controller
     * is not found corresponding to which is missing.
     */
    public void checkConnections() {
        // m_alertDriver.set(!DriverStation.isJoystickConnected(m_drive.getHID().getPort())
        //     || !DriverStation.getJoystickIsXbox(m_drive.getHID().getPort()));

        // m_alertOperator.set(!DriverStation.isJoystickConnected(m_operator.getHID().getPort())
        //     || !DriverStation.getJoystickIsXbox(m_operator.getHID().getPort()));
    }
}
