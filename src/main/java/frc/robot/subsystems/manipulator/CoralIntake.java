// Copybottom (c) FIRST and other WPILib contributors.
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.shared.Constants;
import java.util.function.DoubleSupplier;
import frc.robot.shared.Logger;

import frc.robot.subsystems.body.BodyConstants;
import frc.robot.subsystems.body.BodyConstants.Setpoints;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.manipulator.ManipulatorConstants;

public class CoralIntake extends SubsystemBase {
  private static CoralIntake m_instance;
  
  public static CoralIntake getInstance() {
    if(m_instance == null) m_instance = new CoralIntake();
    return m_instance;
  }

  //Defining Motors
  private final SparkMax m_topMotor;
  private final SparkMax m_bottomMotor;
  private SparkMaxConfig m_topMotorConfig;
  private SparkMaxConfig m_bottomMotorConfig;

  private DigitalInput m_lowBrake;
  private DigitalInput m_highBrake;

  private final int kLowBrakeId = 1;
  private final int kHighBrakeId = 4;

  private double ShotSpeed = 1;

  private final Elevator m_Elevator = Elevator.getInstance();

  private CoralIntake() {
    //Defining Motors part 2
    m_topMotor = new SparkMax(ManipulatorConstants.kCoralIntakeConfig.topId(), MotorType.kBrushless);
    m_bottomMotor = new SparkMax(ManipulatorConstants.kCoralIntakeConfig.bottomId(), MotorType.kBrushless);
    m_topMotorConfig = new SparkMaxConfig();
    m_bottomMotorConfig = new SparkMaxConfig();

    m_topMotorConfig.idleMode(IdleMode.kCoast);
    m_topMotorConfig.smartCurrentLimit(30);
    m_bottomMotorConfig.apply(m_topMotorConfig);

    m_topMotor.configure(m_topMotorConfig, null,  PersistMode.kPersistParameters);
    m_bottomMotor.configure(m_bottomMotorConfig, null, PersistMode.kPersistParameters);

    m_lowBrake = new DigitalInput(kLowBrakeId);
    m_highBrake = new DigitalInput(kHighBrakeId);
  }

    /**
   * Sets the percent output on the top and bottom intake roller.
   * @param percent The percent to set. Value should be between -1.0 and 1.0
   */
  public void setPercent(double bpercent, double tpercent) {
    var m_topMotorFaults = m_topMotor.getFaults();
    var m_bottomMotorFaults = m_bottomMotor.getFaults();
    // Logger.println("Speed: " + m_topMotor.getAppliedOutput());
    // Logger.println("Encoder Position: " + m_Elevator.encoderPosition());
    // Logger.println("Calculated Shot Speed: " + getShotSpeed());
    if (m_topMotorFaults.temperature != true && m_bottomMotorFaults.temperature != true) {
      m_topMotor.set(tpercent);
      m_bottomMotor.set(bpercent);    
    } else {
      System.out.println("Overheating! Stop! (Temp Fault - Top Motor): " + m_topMotorFaults.temperature + "(Temp Fault - Bottom Motor): " + m_bottomMotorFaults.temperature);
    }
  }

  public Command runCoralIntake(double bPercent, double tPercent) {
    CoralIntake m_CoralIntake = CoralIntake.getInstance();
    return new RunCommand(()->m_CoralIntake.setPercent(bPercent, tPercent));
  }

  public Command stopCoralIntake(){
    CoralIntake m_CoralIntake = CoralIntake.getInstance();
    return new InstantCommand(()-> m_CoralIntake.stop());
  }

  /**
   * Stops both intake rollers.
   */
  public void stop() {
    m_topMotor.set(0.0);
    m_bottomMotor.set(0.0);
  }

  public boolean lowBrakeTrigger() {
    return !m_lowBrake.get();
  }

  public boolean highBrakeTrigger() {
    return !m_highBrake.get();
  }


  public double getShotSpeedBottom(){
    if(m_Elevator.encoderPosition() < BodyConstants.L1_ELEVATOR_INCHES + 0.25){
      return 0.6; // 0.05 - 0.2
    } 
    if(m_Elevator.encoderPosition() < BodyConstants.L2_ELEVATOR_INCHES + 0.25){
      return 0.8; // 0.05 - 0.2
    } 
    if(m_Elevator.encoderPosition() < BodyConstants.L3_ELEVATOR_INCHES + 0.25){
      return 0.8; // 0.05 - 0.2
    } 
    return 1;
  }

  public double getShotSpeedTop(){
    if(m_Elevator.encoderPosition() < BodyConstants.L1_ELEVATOR_INCHES + 0.25){
      return 0.41; // 0.05 - 0.2
    }
    if(m_Elevator.encoderPosition() < BodyConstants.L2_ELEVATOR_INCHES + 0.25){
      return 0.75; // 0.05 - 0.2
    } 
    if(m_Elevator.encoderPosition() < BodyConstants.L3_ELEVATOR_INCHES + 0.25){
      return 0.75; // 0.05 - 0.2
    } 
    return 1;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("CoralIntake");

    if(Constants.Dashboard.kSendStates) {
      builder.addBooleanProperty("Low Brake", this::lowBrakeTrigger, null);
      builder.addBooleanProperty("High Brake", this::highBrakeTrigger, null);
    }

  }
}
