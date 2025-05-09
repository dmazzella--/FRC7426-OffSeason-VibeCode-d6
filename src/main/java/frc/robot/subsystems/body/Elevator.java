// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.body;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.shared.Constants;
import frc.robot.shared.Limelight;
import frc.robot.shared.Logger;
import frc.robot.subsystems.ReconfigurableConfig;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.swerve.TunerConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.subsystems.body.BodyConstants;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase implements ReconfigurableConfig {
  private static Elevator m_instance;
  
  public static Elevator getInstance() {
    if(m_instance == null) m_instance = new Elevator();
    return m_instance;
  }

  //Setting Motors
  private TalonFX m_leftMotor;
  private TalonFX m_rightMotor;
  private CANcoder m_elevatorEncoder;

  private final MotionMagicVoltage m_positionOut;

  private DigitalInput m_limitSwitch;

  private final int kLimitSwitchId = 0;

  private Setpoints m_setpoint;

  private double newElMotorVoltage;
  
  public static double SLOT_0_P = 10;
  public static double SLOT_0_I = 0;
  public static double SLOT_0_D = 0.1;

  public static double SLOT_1_P = 50;
  public static double SLOT_1_I = 0;
  public static double SLOT_1_D = 2.5;

  public static double SLOT_2_P = 5;
  public static double SLOT_2_I = 0;
  public static double SLOT_2_D = 0.05;

  public static double MOTIONMAGIC_ACCELERATION = 600;
  public static double MOTIONMAGIC_VELOCITY = 30;
  public static double MOTIONMAGIC_JERK = 0;

  private Elevator() {
    //Setting Motors and Encoder 
    m_leftMotor = new TalonFX(BodyConstants.kElevatorConfig.leftId(), "");
    m_rightMotor = new TalonFX(BodyConstants.kElevatorConfig.rightId(), "");
    m_elevatorEncoder = new CANcoder(BodyConstants.kElevatorConfig.encoderId(), "");

    // Creating new control modes
    m_positionOut = new MotionMagicVoltage(0.0).withSlot(0);

    m_limitSwitch = new DigitalInput(kLimitSwitchId);

    configMotor(m_leftMotor.getConfigurator(), m_elevatorEncoder);

    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));

    m_setpoint = Setpoints.IdleHigh;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Max Speed", TunerConstants.MaxSpeed);
    SmartDashboard.putNumber("Max Angular Rate", TunerConstants.MaxAngularRate);
    // Logger.println("Auto Stage: " + Limelight.autoStage);
    moveToSetpoint();
    if(encoderPosition() > 9){
      TunerConstants.MaxSpeed = 4.74 / (encoderPosition() * 0.55);
      TunerConstants.MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) / (encoderPosition() * 0.55);
    } else {
      TunerConstants.MaxSpeed = 4.74;
      TunerConstants.MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }
    if (m_setpoint.equals(Setpoints.L4Setpoint)) {
      Limelight.MaxSpeed = 1 * 0.65; //0.5
    } else {
      Limelight.MaxSpeed = 1;
    }
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
      encoderConfig.MagnetSensor.MagnetOffset = -0.41845703125;

      encoder.getConfigurator().apply(encoderConfig, 0.05);
    }
    
    // Set soft limits
    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = BodyConstants.kElevatorLimits.forwardLimit();
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = BodyConstants.kElevatorLimits.reverseLimit();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // -0.15 to go down
    // 0.16 to go up

    // Set max voltage
    var voltage = newConfig.Voltage; 
    voltage.PeakForwardVoltage = 13; //16
    voltage.PeakReverseVoltage = -13; //16

    // Set ramp period (0.02 - 0.05 secs)
    var ramp = newConfig.ClosedLoopRamps;
    ramp.VoltageClosedLoopRampPeriod = 0.05;

    // Set current limits
    // var current = newConfig.CurrentLimits;
    // current.StatorCurrentLimit = BodyConstants.kWristLimits.statorLimit();
    // current.StatorCurrentLimitEnable = true;
    // current.SupplyCurrentLimit = BodyConstants.kWristLimits.supplyLimit();
    // current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0 (Idle Slots)
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = SLOT_0_P; //55
    slot0.kI = SLOT_0_I;
    slot0.kD = SLOT_0_D; 

    // Configure PID in Slot 1 (Low Slot)
    Slot1Configs slot1 = newConfig.Slot1;
    slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot1.kP = SLOT_1_P; //25
    slot1.kI = SLOT_1_I;
    slot1.kD = SLOT_1_D; 

    // Configure PID in Slot 2 (Algae Slots)
    Slot2Configs slot2 = newConfig.Slot2;
    slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot2.kP = SLOT_2_P; //55
    slot2.kI = SLOT_2_I;
    slot2.kD = SLOT_2_D;

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicAcceleration = MOTIONMAGIC_ACCELERATION;
    motionMagic.MotionMagicCruiseVelocity = MOTIONMAGIC_VELOCITY;
    motionMagic.MotionMagicJerk = MOTIONMAGIC_JERK;

    var motorOutput = newConfig.MotorOutput;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  public boolean limitSwitchTrigger() {
    return !m_limitSwitch.get();
  }

  /**
   * Updates the current setpoint for the elevator to reference.
   * @param setpoint A pre-determined setpoint.
   */
  public void updateSetpoint(Setpoints setpoint) {
    m_setpoint = setpoint;
  }
  
  public void moveToSetpoint() {
    if (m_setpoint != null) {
      m_leftMotor.setControl(
        m_positionOut
          .withPosition(m_setpoint.getInches() / 4.25)
          .withSlot(m_setpoint.getElevatorSlot())
          .withFeedForward(m_setpoint.getElevatorFeed()));
    }
  }

  public double getReference() {
    return m_setpoint.getInches();
  }

  public double encoderPosition() {
    return (m_elevatorEncoder.getPosition().getValueAsDouble()) * 4.25; //4.75 - 4.85
  }

  public double getError() {
    return encoderPosition() - m_setpoint.getInches();
  }

  public void setVoltage(double newVoltage) {
    newElMotorVoltage = newVoltage;
    m_leftMotor.setVoltage(newElMotorVoltage);
  }

 public void stopMotor(){
    m_leftMotor.stopMotor();
  }

  public double getVoltage() {
    return newElMotorVoltage;
  }

  //LOWERED ALL ISL VALUES DUE TO CHANGES IN ELEVATOR RIGGING

  public boolean isL4(){
    if (encoderPosition() > BodyConstants.L4_ELEVATOR_INCHES - 1) {
      return true;
    }
    return false;
  }

  public boolean isL3(){
    if (encoderPosition() > BodyConstants.L3_ELEVATOR_INCHES - 1) {
      return true;
    }
    return false;
  }

  public boolean isL2(){
    if (encoderPosition() > BodyConstants.L2_ELEVATOR_INCHES - 1) {
      return true;
    }
    return false;
  }


  public boolean isIdle(){
    if (encoderPosition() < BodyConstants.IDLE_ELEVATOR_INCHES + 1.5) {
      return true;
    }
    return false;
  }

  public boolean isIdleHigh(){
    if (encoderPosition() < BodyConstants.IDLE_HIGH_ELEVATOR_INCHES + 3) {
      return true;
    }
    return false;
  }

  public boolean isClimbHigh(){
    if (encoderPosition() >= BodyConstants.CLIMBER_START_ELEVATOR_INCHES - 1.17) {
      return true;
    }
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");
    
    if(Constants.Dashboard.kSendErrors) {
      builder.addDoubleProperty("Error", this::getError, null);
    }
    
    if(Constants.Dashboard.kSendPID) {
      builder.addDoubleProperty("Elevator Motor Voltage", this::getVoltage, this::setVoltage);
      builder.addDoubleProperty("Reference", this::getReference, null);
    }

    if(Constants.Dashboard.kSendStates) {
      builder.addDoubleProperty("Encoder Position", this::encoderPosition, null);
      builder.addBooleanProperty("Limit Switch", this::limitSwitchTrigger, null);
    }

    if(Constants.Dashboard.kSendDebug) {
      builder.addStringProperty("Setpoint Name", ()->{return m_setpoint.name();}, null);
    }

  }

  @Override
  public void reconfigure() {
    configMotor(m_leftMotor.getConfigurator(), m_elevatorEncoder);

    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));

  }
}
