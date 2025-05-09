package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.PIDConfig;

public class SwerveModule {
  // Motors
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  
  // Encoder
  private final CANCoder absoluteEncoder;
  private final double absoluteEncoderOffset;
  
  // PID controllers
  private PIDController turnPIDController;
  
  /**
   * Create a new swerve module
   * 
   * @param driveMotorId CAN ID for drive motor
   * @param turnMotorId CAN ID for turn motor
   * @param encoderId CAN ID for absolute encoder
   * @param encoderOffset Offset for absolute encoder
   * @param drivePID PID configuration for drive motor
   * @param turnPID PID configuration for turn motor
   */
  public SwerveModule(
    int driveMotorId,
    int turnMotorId,
    int encoderId,
    double encoderOffset,
    PIDConfig drivePID,
    PIDConfig turnPID
  ) {
    driveMotor = new TalonFX(driveMotorId);
    turnMotor = new TalonFX(turnMotorId);
    absoluteEncoder = new CANCoder(encoderId);
    absoluteEncoderOffset = encoderOffset;
    
    // Configure drive motor
    driveMotor.configFactoryDefault();
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    // Configure turn motor
    turnMotor.configFactoryDefault();
    turnMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    // Configure drive PID
    driveMotor.config_kP(0, drivePID.kP);
    driveMotor.config_kI(0, drivePID.kI);
    driveMotor.config_kD(0, drivePID.kD);
    driveMotor.config_kF(0, drivePID.kF);
    
    // Configure turn PID controller
    turnPIDController = new PIDController(turnPID.kP, turnPID.kI, turnPID.kD);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Reset encoders
    resetEncoders();
  }
  
  /**
   * Reset the module encoders
   */
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    turnMotor.setSelectedSensorPosition(getAbsoluteEncoderRadians() / Constants.TURN_POSITION_CONVERSION);
  }
  
  /**
   * Get the absolute encoder angle in radians
   * 
   * @return Absolute encoder angle in radians
   */
  public double getAbsoluteEncoderRadians() {
    double angle = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition());
    angle -= Units.degreesToRadians(absoluteEncoderOffset);
    return angle;
  }
  
  /**
   * Get the current state of the module
   * 
   * @return Current state (speed and angle)
   */
  public SwerveModuleState getState() {
    double velocity = driveMotor.getSelectedSensorVelocity() * Constants.DRIVE_VELOCITY_CONVERSION;
    double angle = turnMotor.getSelectedSensorPosition() * Constants.TURN_POSITION_CONVERSION;
    return new SwerveModuleState(velocity, new Rotation2d(angle));
  }
  
  /**
   * Get the current position of the module
   * 
   * @return Current position (distance and angle)
   */
  public SwerveModulePosition getPosition() {
    double distance = driveMotor.getSelectedSensorPosition() * Constants.DRIVE_POSITION_CONVERSION;
    double angle = turnMotor.getSelectedSensorPosition() * Constants.TURN_POSITION_CONVERSION;
    return new SwerveModulePosition(distance, new Rotation2d(angle));
  }
  
  /**
   * Set the desired state of the module
   * 
   * @param desiredState Desired state (speed and angle)
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the state to avoid spinning more than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState,
      new Rotation2d(turnMotor.getSelectedSensorPosition() * Constants.TURN_POSITION_CONVERSION)
    );
    
    // Calculate drive output
    double driveOutput = state.speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND;
    
    // Calculate turn output using PID controller
    double turnOutput = turnPIDController.calculate(
      turnMotor.getSelectedSensorPosition() * Constants.TURN_POSITION_CONVERSION,
      state.angle.getRadians()
    );
    
    // Set motor outputs
    driveMotor.set(ControlMode.PercentOutput, driveOutput);
    turnMotor.set(ControlMode.PercentOutput, turnOutput);
  }
  
  /**
   * Update PID values for the module
   * 
   * @param drivePID New drive PID configuration
   * @param turnPID New turn PID configuration
   */
  public void updatePIDValues(PIDConfig drivePID, PIDConfig turnPID) {
    // Update drive PID
    driveMotor.config_kP(0, drivePID.kP);
    driveMotor.config_kI(0, drivePID.kI);
    driveMotor.config_kD(0, drivePID.kD);
    driveMotor.config_kF(0, drivePID.kF);
    
    // Update turn PID
    turnPIDController.setP(turnPID.kP);
    turnPIDController.setI(turnPID.kI);
    turnPIDController.setD(turnPID.kD);
  }
}