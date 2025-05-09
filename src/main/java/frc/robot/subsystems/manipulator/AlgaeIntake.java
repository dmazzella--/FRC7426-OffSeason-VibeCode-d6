// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.Constants;
import frc.robot.shared.Logger;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.manipulator.ManipulatorConstants;

public class AlgaeIntake extends SubsystemBase {
  private static AlgaeIntake m_instance;
  
  public static AlgaeIntake getInstance() {
    if(m_instance == null) m_instance = new AlgaeIntake();
    return m_instance;
  }

  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;
  private SparkMaxConfig m_leftMotorConfig;
  private SparkMaxConfig m_rightMotorConfig;

  private DigitalInput m_algaeBrake;

  private final int kAlgaeBrakeId = 3;

  private AlgaeIntake() {
    //Defining motors
    m_leftMotor = new SparkMax(ManipulatorConstants.kAlgaeIntakeConfig.leftId(), MotorType.kBrushless);
    m_rightMotor = new SparkMax(ManipulatorConstants.kAlgaeIntakeConfig.rightId(), MotorType.kBrushless);
    m_leftMotorConfig = new SparkMaxConfig();
    m_rightMotorConfig = new SparkMaxConfig();

    m_algaeBrake = new DigitalInput(kAlgaeBrakeId);
    
    m_leftMotorConfig.idleMode(IdleMode.kBrake);
    m_leftMotorConfig.smartCurrentLimit(20);
    m_rightMotorConfig.apply(m_leftMotorConfig);
    m_rightMotorConfig.inverted(true);
    // m_rightMotorConfig.follow(m_leftMotor, true);

    m_leftMotor.configure(m_leftMotorConfig, null,  PersistMode.kPersistParameters);
    m_rightMotor.configure(m_rightMotorConfig, null, PersistMode.kPersistParameters);
}

  /**
   * Sets the percent output on the left and right intake roller.
   * @param percent The percent to set. Value should be between -1.0 and 1.0
   */
  public void setPercent(double leftPercent, double rightPercent) {
    var m_leftMotorFaults = m_leftMotor.getFaults();
    var m_rightMotorFaults = m_rightMotor.getFaults();
    Logger.println("Left Motor Power: " + m_leftMotor.getAppliedOutput());
    Logger.println("Right Motor Power: " + m_rightMotor.getAppliedOutput());
    if (m_leftMotorFaults.temperature != true && m_rightMotorFaults.temperature != true) {
      m_leftMotor.set(leftPercent);
      m_rightMotor.set(rightPercent);    
    } else {
      System.out.println("Overheating! Stop! (Temp Fault - Left Motor): " + m_leftMotorFaults.temperature + "(Temp Fault - Right Motor): " + m_rightMotorFaults.temperature);
    }
  }

  public Command runAlgaeIntake(double leftPercent, double rightPercent) {
    AlgaeIntake m_AlgaeIntake = AlgaeIntake.getInstance();
    return new RunCommand(()->m_AlgaeIntake.setPercent(leftPercent, rightPercent), m_AlgaeIntake);
  }

  public Command stopAlgaeIntake(){
    AlgaeIntake m_AlgaeIntake = AlgaeIntake.getInstance();
    return new InstantCommand(()->m_AlgaeIntake.setPercent(0, 0), m_AlgaeIntake);
  }

  public double getleftVoltage(){
    return m_leftMotor.getAppliedOutput();
  }

  public double getrightVoltage(){
    return m_rightMotor.getAppliedOutput();
  }

  public boolean algaeBrakeTrigger() {
    return !m_algaeBrake.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("AlgaeIntake");
    
    if(Constants.Dashboard.kSendStates) {
      builder.addBooleanProperty("AlgaeIntake Locked", this::algaeBrakeTrigger, null);
    }

    if(Constants.Dashboard.kSendSystemCheck) {
      builder.addDoubleProperty("left AlgaeIntake Wattage", this::getleftVoltage , null);
      builder.addDoubleProperty("right AlgaeIntake Wattage", this::getrightVoltage, null);
    }
  }
}
