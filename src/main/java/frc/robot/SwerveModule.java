// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  //private static final double kWheelRadius = 0.0508;
  //private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  public final TalonSRX m_driveMotor;
  public final TalonSRX m_turningMotor;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.5, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          2,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 0.5);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.05);

  private final double rawTickToDegreesRatio = 360.0/1656.666;
  public final double rawAnalogToDegreesRatio = 360.0/1024.0;
  public double initQuadPosition = -1;
  public double initAnalogPos = -1;
  public double analogOffsetDeg = -1;
  public String name = "noName";
  public double turnOffset = -1;
  public double drivingTicksPerRotToRotations = 1/(256*2*16*6.666);
  public double drivingTicksPer100MsToRotationsPerSecond = 1/(10*256*2*16*6.666);
  public double drivingRotationsPerMeter = (0.1016 * Math.PI);
  public double driveOffset = 1;


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel) {

      
    m_driveMotor = new TalonSRX(driveMotorChannel);
    m_turningMotor = new TalonSRX(turningMotorChannel);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
   
    return new SwerveModuleState(
        getDriveVelocityMPS(), new Rotation2d(this.getTurningMotorPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistMeters(), new Rotation2d(this.getTurningMotorPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(this.getTurningMotorPosition())));
    SmartDashboard.putNumber(name + "Desired Ang", state.angle.getDegrees());
    SmartDashboard.putNumber(name + "Desired Velo", state.speedMetersPerSecond);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDriveVelocityMPS(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(Math.toRadians(this.getTurningMotorPosition()), state.angle.getRadians());

    //final double turnFeedforward =
     //   m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.set(ControlMode.PercentOutput, driveOffset * driveOutput);// + driveFeedforward);
    m_turningMotor.set(ControlMode.PercentOutput, (turnOutput));// + turnFeedforward));
  }

  public double getTurningMotorPosition(){
    double rawPos = -m_turningMotor.getSelectedSensorPosition();
    double _0to360 = (rawPos - initQuadPosition) * rawTickToDegreesRatio + analogOffsetDeg;
    double ret = _0to360 - 360 * Math.floor(_0to360 / 360) - 180;
    return -ret;
    
  }

  public void initModule(){
    m_driveMotor.setSelectedSensorPosition(0);

    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.configFeedbackNotContinuous(true, 0);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog,0,0);
    Timer delay = new Timer();
    delay.reset();
    delay.start();
    while(delay.advanceIfElapsed(0.1) == false){
      //wait
    }

    initAnalogPos = m_turningMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber(name + " Analog Init", initAnalogPos);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);

    delay.reset();
    delay.start();
    while(delay.advanceIfElapsed(0.1) == false){
      //wait
    }

    initQuadPosition = -m_turningMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber(name + " Init Quad", initQuadPosition);

    analogOffsetDeg = (initAnalogPos * rawAnalogToDegreesRatio) + turnOffset;
    SmartDashboard.putNumber(name + " Analog Deg", analogOffsetDeg);
  }

  public void checkUserButton(){
    if(!RobotController.getUserButton()){
      m_turningMotor.setNeutralMode(NeutralMode.Coast);
      m_driveMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      m_turningMotor.setNeutralMode(NeutralMode.Brake);
      m_driveMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public void updateSmartDashboard(){
    SmartDashboard.putNumber(name + "Raw Pos", m_turningMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber(name + "Calculated Pos", getTurningMotorPosition());
    SmartDashboard.putNumber(name + "Raw Velocity",m_driveMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber(name + "Drive Dist", getDriveDistMeters());
    SmartDashboard.putNumber(name + "Drive Rots", getDriveRotations());
    SmartDashboard.putNumber(name + "Drive MPS", getDriveVelocityMPS());

  }

  public double getDriveVelocityMPS(){
    return driveOffset * m_driveMotor.getSelectedSensorVelocity() * drivingTicksPer100MsToRotationsPerSecond * drivingRotationsPerMeter;
  }

  public double getDriveDistMeters(){
    return driveOffset * m_driveMotor.getSelectedSensorPosition() * drivingTicksPerRotToRotations * drivingRotationsPerMeter;
  }

  public double getDriveRotations(){
    return driveOffset * m_driveMotor.getSelectedSensorPosition() * drivingTicksPerRotToRotations;
  }
} 
