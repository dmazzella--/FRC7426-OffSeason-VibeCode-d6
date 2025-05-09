// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.body;

import frc.robot.subsystems.ReconfigurableConfig;

public class BodyConstants implements ReconfigurableConfig {

    // public static int kElevatorCurrentLimit = 100;
    public static double kWristStatorLimit = 80.0;
    public static double kWristSupplyLimit = 45.0;
    public static double kWristFowardLimit = 10.0;   
    public static double kWristReverseLimit = -130.0;
    public static double kElevatorFowardLimit = 23.0;   
    public static double kElevatorReverseLimit = 0;

    public static final ElevatorConfig kElevatorConfig = new ElevatorConfig(15, 16, 17);
    public static ElevatorLimits kElevatorLimits = new ElevatorLimits(kWristStatorLimit, kWristSupplyLimit, kElevatorFowardLimit, kElevatorReverseLimit);

    public static final WristConfig kWristConfig = new WristConfig(18, 20);
    public static WristLimits kWristLimits = new WristLimits(kWristStatorLimit, kWristSupplyLimit, kWristFowardLimit, kWristReverseLimit);

    public record ElevatorConfig(int leftId, int rightId, int encoderId) {};
    public record WristConfig(int motorId, int encoderId) {};
    public record ElevatorLimits(double statorLimit, double supplyLimit, double forwardLimit, double reverseLimit) {};
    public record WristLimits(double statorLimit, double supplyLimit, double forwardLimit, double reverseLimit) {};

    /**
     * An enum used to store the information needed for each setpoint relating to the Wrist and elevator.
     *
     * @param wristDegrees The set position for the wrist to go to in degrees.
     * @param wristFeed The set feed forward for the wrist to use in setpoints.
     * @param wristNewSlot The slot to use for the wrist
     * @param elevatorInches The set position for the elevator to use in inches.
     * @param elevatorFeed The set feed forward for the elevator to use in setpoints.
     */
    public static double IDLE_WRIST_POSITION = 10;
    public static double IDLE_HIGH_WRIST_POSITION = 97;
    public static double L1_WRIST_POSITION = 70;
    public static double L2_WRIST_POSITION = 110;
    public static double L3_WRIST_POSITION = 110;
    public static double L4_WRIST_POSITION = 121; 
    public static double HUMAN_PLAYER_WRIST_POSITION = 39;
    public static double HIGH_ALGAE_WRIST_POSITION = 107;
    public static double LOW_ALGAE_WRIST_POSITION = 107; 
    public static double GROUND_ALGAE_WRIST_POSITION = 110; 
    public static double BARGE_WRIST_POSITION = 35;
    public static double BARGE_BACK_WRIST_POSITION = -20;
    public static double PROCESSOR_WRIST_POSITION = 100;
    public static double ELEVATOR_TEST_WRIST_POSITION = 0;
    public static double CLIMBER_START_WRIST_POSITION = 10; 
    public static double CLIMBER_END_WRIST_POSITION = 105;  

    public static double IDLE_ELEVATOR_INCHES = 2.165; 
    public static double IDLE_HIGH_ELEVATOR_INCHES = 6.645;
    public static double L1_ELEVATOR_INCHES = 1.215;
    public static double L2_ELEVATOR_INCHES = 6.794; 
    public static double L3_ELEVATOR_INCHES = 11.165; 
    public static double L4_ELEVATOR_INCHES = 19.028;
    public static double HUMAN_PLAYER_ELEVATOR_INCHES = 2.935; 
    public static double HIGH_ALGAE_ELEVATOR_INCHES = 12.392;
    public static double LOW_ALGAE_ELEVATOR_INCHES = 8.115; 
    public static double GROUND_ALGAE_ELEVATOR_INCHES = -0.08; 
    public static double BARGE_ELEVATOR_INCHES = 18.945; // 20.92
    public static double BARGE_BACK_ELEVATOR_INCHES = 18.945; // 20.92
    public static double PROCESSOR_ELEVATOR_INCHES = 2.765;
    public static double ELEVATOR_TEST_ELEVATOR_INCHES = 2.845;
    public static double CLIMBER_START_ELEVATOR_INCHES = 8.515;
    public static double CLIMBER_END_ELEVATOR_INCHES = 0.265;

    public enum Setpoints {
        // Wrist Setpoint
        Idle(IDLE_WRIST_POSITION, 0, 2, IDLE_ELEVATOR_INCHES, 0, 0),
        AlgaeIdle(IDLE_WRIST_POSITION, 0, 2, IDLE_ELEVATOR_INCHES, 0, 2),
        IdleHigh(IDLE_HIGH_WRIST_POSITION, 0, 1, IDLE_HIGH_ELEVATOR_INCHES, 0, 0),
        L1Setpoint(L1_WRIST_POSITION, 0, 0, L1_ELEVATOR_INCHES, 0, 0),
        L2Setpoint(L2_WRIST_POSITION, 0, 0, L2_ELEVATOR_INCHES, 0, 1),
        L3Setpoint(L3_WRIST_POSITION, 0, 0, L3_ELEVATOR_INCHES, 0, 1),
        L4Setpoint(L4_WRIST_POSITION, 0, 0, L4_ELEVATOR_INCHES, 0, 1),
        HumanPlayerSetpoint(HUMAN_PLAYER_WRIST_POSITION, 0, 1, HUMAN_PLAYER_ELEVATOR_INCHES, 0, 2),
        HighAlgaeSetpoint(HIGH_ALGAE_WRIST_POSITION, 0, 0, HIGH_ALGAE_ELEVATOR_INCHES, 0, 1),
        LowAlgaeSetpoint(LOW_ALGAE_WRIST_POSITION, 0, 0, LOW_ALGAE_ELEVATOR_INCHES, 0, 1),
        GroundAlgaeSetpoint(GROUND_ALGAE_WRIST_POSITION, 0, 0, GROUND_ALGAE_ELEVATOR_INCHES,0, 1),
        Barge(BARGE_WRIST_POSITION, 0, 2, BARGE_ELEVATOR_INCHES, 0, 1),
        BargeBack(BARGE_BACK_WRIST_POSITION, 0, 2, BARGE_BACK_ELEVATOR_INCHES, 0, 1),
        Processor(PROCESSOR_WRIST_POSITION, 0, 2, PROCESSOR_ELEVATOR_INCHES, 0, 1),
        ClimberStart(CLIMBER_START_WRIST_POSITION, 0, 0, CLIMBER_START_ELEVATOR_INCHES, 0, 1),
        ClimberEnd(CLIMBER_END_WRIST_POSITION, 0, 2, CLIMBER_END_ELEVATOR_INCHES, 0, 2),
        ElevatorTest(ELEVATOR_TEST_WRIST_POSITION, 0, 0, ELEVATOR_TEST_ELEVATOR_INCHES, 0, 1);

        private double wristDegrees;
        private final double wristFeedForward;
        private final int wristSlot;
        private double elevatorPosition;
        private final int elevatorNewSlot;
        private final double elevatorFeedForward;

        Setpoints(double wristPosition, double wristFeed, int wristNewSlot, double elevatorInches, double elevatorFeed, int elevatorSlot) {
            this.wristDegrees = wristPosition;
            this.wristFeedForward = wristFeed;
            this.wristSlot = wristNewSlot;
            this.elevatorPosition = elevatorInches;
            this.elevatorFeedForward = elevatorFeed;
            this.elevatorNewSlot = elevatorSlot;
        }

        public void setWristPosition(double wristPosition) {
            this.wristDegrees = wristPosition;
        }

        public void setElevatorPosition(double elevatorPosition) {
            this.elevatorPosition = elevatorPosition;
        }
        /**
         * Gets the desired inches of the elevator for the given setpoint
         * @return Elevator inches
         */
        public double getInches() {
            return this.elevatorPosition;
        }
        
        /**
         * Gets the desired degrees of the wrist for the given setpoint
         * @return Wrist degrees
         */
        public double getDegrees() {
            return this.wristDegrees;
        }

        /**
         * Gets the desired feed forward to the wrist for the given setpoint
         * @return Wrist feed forward in volts
         */
        public double getWristFeed() {
            return this.wristFeedForward;
        }

        /**
         * Gets the desired feed forward to the elevator for the given setpoint
         * @return Elevator feed forward in volts
         */
        public double getElevatorFeed() {
            return this.elevatorFeedForward;
        }
        
        /**
         * Gets the desired feed forward to the wrist for the given setpoint
         * @return Wrist feed forward in volts
         */
        public int getWristSlot() {
            return this.wristSlot;
        }

        /**
         * Gets the desired slot to the elevator for the given setpoint
         * @return Elevator slot
         */
        public int getElevatorSlot() {
            return this.elevatorNewSlot;
        }

    }

    @Override
    public void reconfigure() {
        kWristLimits = new WristLimits(kWristStatorLimit, kWristSupplyLimit, kWristFowardLimit, kWristReverseLimit);
        // kElevatorLimits = new ElevatorLimits(kElevatorCurrentLimit);
        Setpoints.Idle.setWristPosition(IDLE_WRIST_POSITION);
        Setpoints.Idle.setElevatorPosition(IDLE_ELEVATOR_INCHES);
        Setpoints.L2Setpoint.setWristPosition(L2_WRIST_POSITION);
        Setpoints.L2Setpoint.setElevatorPosition(L2_ELEVATOR_INCHES);
        Setpoints.L3Setpoint.setWristPosition(L3_WRIST_POSITION);
        Setpoints.L3Setpoint.setElevatorPosition(L3_ELEVATOR_INCHES);
        Setpoints.L4Setpoint.setWristPosition(L4_WRIST_POSITION);
        Setpoints.L4Setpoint.setElevatorPosition(L4_ELEVATOR_INCHES);
        Setpoints.HumanPlayerSetpoint.setWristPosition(HUMAN_PLAYER_WRIST_POSITION);
        Setpoints.HumanPlayerSetpoint.setElevatorPosition(HUMAN_PLAYER_ELEVATOR_INCHES);
        Setpoints.HighAlgaeSetpoint.setWristPosition(HIGH_ALGAE_WRIST_POSITION);
        Setpoints.HighAlgaeSetpoint.setElevatorPosition(HIGH_ALGAE_ELEVATOR_INCHES);
        Setpoints.LowAlgaeSetpoint.setWristPosition(LOW_ALGAE_WRIST_POSITION);
        Setpoints.LowAlgaeSetpoint.setElevatorPosition(LOW_ALGAE_ELEVATOR_INCHES);
        Setpoints.Barge.setWristPosition(BARGE_WRIST_POSITION);
        Setpoints.Barge.setElevatorPosition(BARGE_ELEVATOR_INCHES);
        Setpoints.Processor.setWristPosition(PROCESSOR_WRIST_POSITION);
        Setpoints.Processor.setElevatorPosition(PROCESSOR_ELEVATOR_INCHES);
        Setpoints.ElevatorTest.setWristPosition(ELEVATOR_TEST_WRIST_POSITION);
        Setpoints.ElevatorTest.setElevatorPosition(ELEVATOR_TEST_ELEVATOR_INCHES);
    }
    

}
