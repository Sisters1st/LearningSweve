// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false);
    
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(m_controller.getLeftX()) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  @Override
  public void robotInit(){
  }

  @Override
  public void robotPeriodic(){
    //1667 counts per revolution
    dashboardUpdate();
    m_swerve.checkRioButtonForCoastMode();

   }

   public void dashboardUpdate(){
    SmartDashboard.putNumber("front right raw pos", m_swerve.m_frontRight.m_turningMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("front left raw pos", m_swerve.m_frontLeft.m_turningMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("front right position", m_swerve.m_frontRight.getTurningMotorPosition());
    SmartDashboard.putNumber("front left velocity", m_swerve.m_frontLeft.m_driveMotor.getSelectedSensorVelocity() );
    SmartDashboard.putNumber("front left position", m_swerve.m_frontLeft.getTurningMotorPosition());
    SmartDashboard.putNumber("back left velocity", m_swerve.m_backLeft.m_driveMotor.getSelectedSensorVelocity() );
    SmartDashboard.putNumber("back left position", m_swerve.m_backLeft.getTurningMotorPosition());
    SmartDashboard.putNumber("front right velocity", m_swerve.m_frontRight.m_driveMotor.getSelectedSensorVelocity() );
    SmartDashboard.putNumber("back right velocity", m_swerve.m_backRight.m_driveMotor.getSelectedSensorVelocity() );
    SmartDashboard.putNumber("back right position", m_swerve.m_backRight.getTurningMotorPosition());

   }

   
}
