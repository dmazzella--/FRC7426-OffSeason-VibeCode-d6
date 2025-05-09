package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxOperator {
    
    protected static CommandXboxController m_operator = new CommandXboxController(OI.OPERATOR_CONTROLLER_ID);

    public static class Setpoint {
        public static Trigger Idle = m_operator.a();
        public static Trigger L1Setpoint = new Trigger(()->false); //Add
        public static Trigger L2Setpoint = m_operator.x();
        public static Trigger L3Setpoint = m_operator.b();
        public static Trigger L4Setpoint = m_operator.y();
        public static Trigger HumanPlayerSetpoint = m_operator.leftTrigger();
        public static Trigger TopAlgae = m_operator.povUp(); //Tune
        public static Trigger BottomAlgae = m_operator.povDown(); //Tunre
        public static Trigger Barge = m_operator.button(8);
        public static Trigger Processor = m_operator.povRight(); //Add
    }

    public final class Climber {
        public static Trigger Push = m_operator.povLeft();
    }

    public final class Drivetrain {
        public static Trigger Brake = new Trigger(()->false);
        public static Trigger ResetOdometry = new Trigger(()->false);
    }

    public final class Intake {
        public static Trigger CoralBeam = m_operator.button(7);
        public static Trigger AlgaeIn = m_operator.leftBumper();
        public static Trigger AlgaeOut = m_operator.rightBumper();
    }
}