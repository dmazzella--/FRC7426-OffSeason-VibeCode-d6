package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DeadpoolPanel {
    // Controller IDs corresponding to DriverStation ports

    protected static CommandJoystick m_operatorUSB = new CommandJoystick(OI.OPERATOR_CONTROLLER_ID);
    
    public final class Intake {
        public static Trigger HumanPlayerOn = m_operatorUSB.button(15);
        public static Trigger HumanPlayerOff = m_operatorUSB.button(12);
        public static Trigger CoralBeam = m_operatorUSB.button(13);
        public static Trigger AlgaeIntakeHigh = m_operatorUSB.button(16);
        public static Trigger AlgaeIntakeLow = m_operatorUSB.button(17);
        public static Trigger AlgaeIntakeGround = m_operatorUSB.button(18);
    }

    public static class Score {
        // Put these in reverse order to match the order of the buttons on the controller
        // reading from the top down, then from left column to right column
        public static Trigger L4Score = m_operatorUSB.button(6);
        public static Trigger L3Score = m_operatorUSB.button(7);
        public static Trigger L2Score = m_operatorUSB.button(8);
        public static Trigger L1Score = m_operatorUSB.button(9);

        public static Trigger R4Score = m_operatorUSB.button(2);
        public static Trigger R3Score = m_operatorUSB.button(3);
        public static Trigger R2Score = m_operatorUSB.button(4);
        public static Trigger R1Score = m_operatorUSB.button(5);

        public static Trigger Idle = m_operatorUSB.button(14);
            
        public static Trigger AlgaeProcessor = m_operatorUSB.button(11);
        public static Trigger AlgaeBarge = m_operatorUSB.button(10);
    }

    public final class Climber {
        public static Trigger Out = m_operatorUSB.button(1);
    }

    public Trigger getButtonPressFromInteger(int button) {
        if (button < 12) {
            return m_operatorUSB.button(button);
        } else {
            Trigger trigger = m_operatorUSB.button((button % 12));
            for (int i = 0; i < button / 12; i++) {
                trigger = trigger.and(m_operatorUSB.button(1));
            }
            return trigger;
        }
    }
    public Trigger createTriggerBinaryInteger(int value) {
        Trigger trigger = m_operatorUSB.button(1);
        if(value > 1){
            for (int i = 1; i < 3; i++) {
                if ((value & (1 << i)) != 0) {
                    trigger = trigger.and(m_operatorUSB.button(i + 1));
                }
            }
        }
        return trigger;
    }

}
