// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  public final SwerveModule m_frontLeft = new SwerveModule(4, 7);
  public final SwerveModule m_frontRight = new SwerveModule(1, 5);
  public final SwerveModule m_backLeft = new SwerveModule(2, 6);
  public final SwerveModule m_backRight = new SwerveModule(3, 8);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public Drivetrain() {
    m_gyro.reset();
    initModules();

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    //m_poseEstimator.addVisionMeasurement(
     //   ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
      //      m_poseEstimator.getEstimatedPosition()),
      //  Timer.getFPGATimestamp() - 0.3);
  }

  public void initModules(){
    m_frontLeft.name = "FrontLeft";
    m_frontRight.name = "FrontRight";
    m_backLeft.name = "BackLeft";
    m_backRight.name = "BackRight";

    m_frontRight.turnOffsetDeg = -90.9;
    m_frontLeft.turnOffsetDeg = 93.13;
    m_backLeft.turnOffsetDeg = -19.57;
    m_backRight.turnOffsetDeg = 100.06;

    m_backLeft.driveMult = -1;
    m_frontLeft.driveMult = -1;
    m_backRight.driveMult = 1;
    m_frontRight.driveMult = 1;

    m_frontRight.minAnalog = 10;
    m_frontRight.maxAnalog = 867;
    
    m_frontLeft.minAnalog = 10;
    m_frontLeft.maxAnalog = 872;
    
    m_backRight.minAnalog = 10;
    m_backRight.maxAnalog = 864;
    
    m_backLeft.minAnalog = 11;
    m_backLeft.maxAnalog = 866;


    m_frontRight.initModule();
    m_backRight.initModule();
    m_frontLeft.initModule();
    m_backLeft.initModule();
   }

   public void checkRioButtonForCoastMode(){
      m_frontRight.checkUserButton();
      m_backRight.checkUserButton();
      m_frontLeft.checkUserButton();
      m_backLeft.checkUserButton();
   }

   public void updateDashboard(){
    m_frontRight.updateSmartDashboard();
    m_backRight.updateSmartDashboard();
    m_frontLeft.updateSmartDashboard();
    m_backLeft.updateSmartDashboard();
   }

}
