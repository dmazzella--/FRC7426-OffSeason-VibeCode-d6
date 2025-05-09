// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.body;

import frc.robot.shared.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.Constants;
import frc.robot.subsystems.body.BodyConstants.Setpoints;

public class Wrist extends SubsystemBase {
  private static Wrist m_instance;
  
  public static Wrist getInstance() {
    if(m_instance == null) m_instance = new Wrist();
    return m_instance;
  }

  private final TalonFX m_Motor;
  private final CANcoder m_encoder;

  public Setpoints m_setpoint;

  private Orchestra m_Orchestra;

  // private WristState m_WristState;

  private double newMotorVoltage;

  private final MotionMagicVoltage m_positionOut;

  // public static enum WristState {
  //   Setpoint,
  //   ClosedLoop;
  // }

  private Wrist() {
    // Creating new motors and encoder
    m_Motor = new TalonFX(BodyConstants.kWristConfig.motorId(), "");
    m_encoder = new CANcoder(BodyConstants.kWristConfig.encoderId(), "");

    // Creating new control modes
    m_positionOut = new MotionMagicVoltage(0.0).withSlot(0);

    // Setting default state and setpoint
    // Configuring motors
    configMotor(m_Motor.getConfigurator(), m_encoder);

    // m_WristState = WristState.Setpoint;
    m_setpoint = Setpoints.IdleHigh;

    // Set bottom motor to follow the top
    // m_bottomMotor.setControl(new Follower(m_topMotor.getDeviceID(), false));

    // Optimize bus utilization
    // m_topMotor.optimizeBusUtilization();
    // m_bottomMotor.optimizeBusUtilization();

  }
  private static void configMotor(TalonFXConfigurator config, CANcoder encoder) {
    var newConfig = new TalonFXConfiguration();

    if(encoder != null) {
      // Set feedback sensor to CANCoder
      FeedbackConfigs feedback = newConfig.Feedback;
      feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
      feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

      //Configing the arm encoder
      var encoderConfig = new CANcoderConfiguration();
      encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      encoderConfig.MagnetSensor.MagnetOffset = 0.111083984375;

      encoder.getConfigurator().apply(encoderConfig, 0.05);
    }
    
    // Set soft limits
    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = BodyConstants.kWristLimits.forwardLimit();
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = BodyConstants.kWristLimits.reverseLimit();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // -0.15 to go down
    // 0.16 to go up

    // Set max voltage
    var voltage = newConfig.Voltage; 
    voltage.PeakForwardVoltage = 6; //16
    voltage.PeakReverseVoltage = -6; //16

    // Set ramp period (0.02 - 0.05 secs)
    var ramp = newConfig.ClosedLoopRamps;
    ramp.VoltageClosedLoopRampPeriod = 0.05;

    // Set current limits
    // var current = newConfig.CurrentLimits;
    // current.StatorCurrentLimit = BodyConstants.kWristLimits.statorLimit();
    // current.StatorCurrentLimitEnable = true;
    // current.SupplyCurrentLimit = BodyConstants.kWristLimits.supplyLimit();
    // current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0 (Other Slots)
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = 30; //55
    slot0.kI = 0;
    slot0.kD = 0.25; 
    slot0.kS = 0.15;
    slot0.kG = 0.16;
    slot0.kV = 4; //1.6

    // Configure PID in Slot 1 (Idle Slot)
    Slot1Configs slot1 = newConfig.Slot1;
    slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot1.kP = 25; //25
    slot1.kI = 0;
    slot1.kD = 0.1; //0.1
    slot0.kS = 0.15;
    slot0.kG = 0.16;
    slot1.kV = 4; //1.6

    // Configure PID in Slot 2 (Algae Slot)
    Slot2Configs slot2 = newConfig.Slot2;
    slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot2.kP = 20; //25
    slot2.kI = 0;
    slot2.kD = 0.05; //0.1
    slot0.kS = 0.15;
    slot0.kG = 0.16;
    slot2.kV = 4; //1.6

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicAcceleration = 900;
    motionMagic.MotionMagicCruiseVelocity = 30;
    motionMagic.MotionMagicJerk = 9999;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  @Override
  public void periodic() {
    moveToSetpoint();
  }

  public Command playMusic(String chrpName) {
    Logger.println("I play music!");
    return new RunCommand(()->m_Orchestra.play());
  }

  public Command pauseMusic(String chrpName) {
    return new RunCommand(()->m_Orchestra.pause());
  }

  /**
   * Moves the Wrist motors toward the current reference.
   */
  public void moveToSetpoint() {
    if (m_setpoint != null) {
      m_Motor.setControl(
        m_positionOut
          .withPosition(m_setpoint.getDegrees() / 360)
          .withSlot(m_setpoint.getWristSlot())
          .withFeedForward(m_setpoint.getWristFeed()));
    }
  }

  public void setVoltage(double newVoltage) {
    newMotorVoltage = newVoltage;
    m_Motor.setControl(new VoltageOut(newVoltage));
  }

  public double getVoltage() {
    return newMotorVoltage;
  }

  /**
   * Updates the current setpoint for the elevator to reference.
   * @param setpoint A pre-determined setpoint.
   */
  public void updateSetpoint(Setpoints setpoint) {
    m_setpoint = setpoint;
  }

  /**
   * @return Rotations of the absolute CANCoder on the Wrist.
   */
  public double getRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }
  
  /**
   * @return Degrees calculated from the current CANCoder rotations.
   */
  public double getDegrees() {
    return getRotations() * 360;
  }

  /**
   * @return The desired Wrist position in degrees.
   */
  public double getReference() {
    return m_setpoint.getDegrees();
  }
  
  /**
   * @return The error between the actual position versus the desired position, in degrees.
   */
  public double getError() {
    return getDegrees() - getReference();
  }

  /**
   * @return Voltage of Top Wrist Motor 
   */
  public double getMotorVoltage(){
    return m_Motor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * @return Current/Amps of Top Wrist Motor 
   */
  public double getMotorCurrent(){
    return m_Motor.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * @return Watts of Top Wrist (Voltage * Amps)
   */
  public double getWatts(){
    return getMotorVoltage() * getMotorCurrent();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Wrist");
    
    if(Constants.Dashboard.kSendErrors) {
      builder.addDoubleProperty("Error", this::getError, null);
    }
    
    if(Constants.Dashboard.kSendPID) {
      builder.addDoubleProperty("Wrist Motor Voltage", this::getVoltage, this::setVoltage);
      builder.addDoubleProperty("Reference", this::getReference, null);
    }

    if(Constants.Dashboard.kSendStates) {
      builder.addDoubleProperty("Position", this::getDegrees, null);
    }

    if(Constants.Dashboard.kSendDebug) {
      builder.addStringProperty("Setpoint Name", ()->{return m_setpoint.name();}, null);
    }

    if(Constants.Dashboard.kSendSystemCheck) {
      builder.addDoubleProperty("Wrist Wattage", this::getWatts, null);
    }
  }
}
