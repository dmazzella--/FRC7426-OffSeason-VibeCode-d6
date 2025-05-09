// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.Constants;

public class Climber extends SubsystemBase {
  private static Climber m_instance;
  
  public static Climber getInstance() {
    if(m_instance == null) m_instance = new Climber();
    return m_instance;
  }

  private SparkMaxConfig m_MotorConfig;
  
  private final SparkMax m_Motor;
  private RelativeEncoder m_MotorEncoder;

  private static final int kSmallMotorId = 25;
  private static final int kBigMotorId = 26;

  private DigitalInput m_ClimbBrake;
  private static final int kBeamBrakeId = 2;

  private boolean climbStart = false;

  private Climber() {

    m_Motor = new SparkMax(kBigMotorId, MotorType.kBrushless);
    m_MotorConfig = new SparkMaxConfig();
    m_MotorEncoder = m_Motor.getEncoder();

    m_MotorConfig.idleMode(IdleMode.kCoast);
    m_MotorConfig.smartCurrentLimit(45);
    m_MotorConfig.inverted(true);

    m_MotorConfig
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //PID
        .p(10)
        .i(0)
        .d(0)
        .outputRange(-1,1)
        .maxMotion
        //MAXMotion
        .maxVelocity(6000)
        .maxAcceleration(2000) //4250
        .allowedClosedLoopError(0.05);

    m_MotorConfig.apply(m_MotorConfig);
    m_MotorConfig.idleMode(IdleMode.kBrake);
    m_MotorConfig.inverted(true);
    m_MotorConfig.smartCurrentLimit(60);

    m_Motor.configure(m_MotorConfig, null,  PersistMode.kPersistParameters);

    m_ClimbBrake = new DigitalInput(kBeamBrakeId);
  }

  // 143 to come back in
  // 96 to go out
  public double encoderPosition() {
    return m_MotorEncoder.getPosition();
  }


  /**
   * Sets the percent output on the big climb motor.
   * @param percent The percent to set. Value should be between -1.0 and 1.0
   */
  public void setCount(double percent) {
    m_Motor.set(percent);
  }


  public Command runClimber(double percent) {
    Climber m_Climber = Climber.getInstance();
    return new RunCommand(()->m_Climber.setCount(percent));
  }

  /**
   * Stops climb motor.
   */
  public void motorStop() {
    m_Motor.set(0.0);
  }

  public boolean climbBrakeTrigger() {
    return m_ClimbBrake.get();
  }

  public void startClimber(){
    climbStart = true;
  }

  public boolean getClimbStart(){
    return climbStart;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");

    if(Constants.Dashboard.kSendStates) {
      builder.addDoubleProperty("Big Encoder Position", this::encoderPosition, null);
      builder.addBooleanProperty("Climb Brake", this::climbBrakeTrigger, null);
    }
  }

}
