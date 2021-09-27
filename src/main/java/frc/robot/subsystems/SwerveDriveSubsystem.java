// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Utils;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveDriveHardware;
import frc.robot.Robot;
import frc.robot.Utils;


public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveDriveHardware m_swerveDriveHardware;
  private SwerveModule m_frontLeftModule;
  private SwerveModule m_frontRightModule;
  private SwerveModule m_backLeftModule;
  private SwerveModule m_backRightModule;
  private AHRS m_gyro;
  private Field2d m_field;
  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rotationSpeed;
  private boolean m_isFieldRelative;
  private SwerveDriveOdometry m_odometry;
  private ChassisSpeeds m_chassisSpeeds;
  private SwerveDriveKinematics m_kinematics;
  private double time;

  /**
   * Determined by the gyro. Signifies whether or not the robot is
   * <b>physically</b> rotating, aka changing heading
   */
  private boolean m_isTurning;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDriveSubsystem(SwerveDriveHardware swerveHardware) {
    m_frontLeftModule = swerveHardware.frontLeft;
    m_frontRightModule = swerveHardware.frontRight;
    m_backLeftModule = swerveHardware.backLeft;
    m_backRightModule = swerveHardware.backRight;

    m_gyro = swerveHardware.gyro;
    m_swerveDriveHardware = swerveHardware;
    m_kinematics = new SwerveDriveKinematics(m_frontLeftModule.getLocation(), m_frontRightModule.getLocation(),
        m_backLeftModule.getLocation(), m_backRightModule.getLocation());

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
    time = 0;
    m_field = new Field2d();

    // TODO right now, the heading PID controller is in degrees. do we want to
    // switch to radians?
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // odometry code will error on the first tick or two due to the gyro taking
    // longer to start up
    if (time > 10) {
      m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()), m_swerveDriveHardware.frontLeft.getState(),
          m_swerveDriveHardware.frontRight.getState(), m_swerveDriveHardware.backLeft.getState(),
          m_swerveDriveHardware.backRight.getState());
      m_field.setRobotPose(m_odometry.getPoseMeters());
      time = 11;
    }
    time++;
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putString("front Left Swerve Module State", m_swerveDriveHardware.frontLeft.getState().toString());
    SmartDashboard.putNumber("time", time);

    // Robot values
    SmartDashboard.putNumber("gyro angle ", Utils.normalizeAngle(m_gyro.getAngle(), 360));
    SmartDashboard.putNumber("gyro rate ",
        Utils.deadZones(m_gyro.getRate(), Constants.SwerveConstants.GYRO_RATE_DEADZONE));

    // LimeLight values
    SmartDashboard.putNumber("Y Offset", Limelight.getY());
    SmartDashboard.putNumber("X Offset", Limelight.getX());
    SmartDashboard.putNumber("Skew", Limelight.getAngle());
    SmartDashboard.putNumber("Latency ", Limelight.getLatency());
    SmartDashboard.putNumber("Target Area", Limelight.getArea());
    SmartDashboard.putBoolean("Has Target", Limelight.hasTarget());

    // TODO somehow account for static friction, I think?

    // Create chassis speeds object.
    // Convert chassis speeds from field-relative speeds to robot-relative speeds,
    // if needed.
    if (m_isFieldRelative) {
      m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rotationSpeed,
          m_gyro.getRotation2d());
    } else {
      m_chassisSpeeds = new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rotationSpeed);
    }

    // toSwerveModuleState array create it from kinematics
    SwerveModuleState[] swerveModuleStates;
    swerveModuleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    if (Robot.isReal()) {
      for (SwerveModuleState swerveModule : swerveModuleStates) {
        swerveModule.speedMetersPerSecond += (SwerveConstants.TRANSLATIONAL_FRICTION
            * SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
      }
    }
    // Normalizing wheel speeds so that the maximum possible speed isn't exceeded.
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates,
        Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

    if (m_chassisSpeeds.vxMetersPerSecond == 0 && m_chassisSpeeds.vyMetersPerSecond == 0
        && m_chassisSpeeds.omegaRadiansPerSecond == 0) {
      // Only stops swerve module's wheel from driving, preserves module's heading.
      m_frontLeftModule.setDesiredState();
      m_frontRightModule.setDesiredState();
      m_backLeftModule.setDesiredState();
      m_backRightModule.setDesiredState();
    } else {
      // swerve module states are given in the same order that the wheels are
      // given to the kinematics object.
      m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
      m_frontRightModule.setDesiredState(swerveModuleStates[1]);
      m_backLeftModule.setDesiredState(swerveModuleStates[2]);
      m_backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    // calculates the rotation of the robot using the desired rotation speed to feed
    // into the simulated gyro.
    // this doesn't affect anything when it gets run on the robot.
    double m_degreeRotationSpeed = Math.toDegrees(m_rotationSpeed);
    double m_degreesSinceLastTick = m_degreeRotationSpeed * Robot.kDefaultPeriod;
    printSimulatedGyro(m_gyro.getYaw() + m_degreesSinceLastTick);

    SmartDashboard.putBoolean("is turning ", m_isTurning);

    SmartDashboard.putNumber("OdometryX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("OdometryY", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("OdometryRot", m_odometry.getPoseMeters().getRotation().getDegrees());
  }

  /**
   * 
   * @param xSpeed          Represents forward velocity w.r.t the robot frame of
   *                        reference. Meters per second, forward is positive
   * @param ySpeed          Represents sideways velocity w.r.t the robot frame of
   *                        reference. Meters per second, <b>left is positive</b>
   * @param rotationSpeed   Represents the angular velocity of the robot frame.
   *                        Radians per second, <b>counterclockwise is
   *                        positive</b>
   * @param isFieldRelative Whether or not the provided x and y values should be
   *                        considered relative to the field, or relative to the
   *                        robot.
   */
  public void move(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative) {
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rotationSpeed = rotationSpeed;
    m_isFieldRelative = isFieldRelative;
    m_isTurning = rotationSpeed != 0;
  }

  /**
   * Resets the gyro. Note that whatever command calls this method <b>will require
   * the subsystem</b>. So no other commands can run on the subsystem while the
   * gyro is being reset, including any drive control commands.
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Sets each of the swerve modules' drive motors to the specified idle state.
   * 
   * @param mode Brake or coast.
   */
  public void setDriveIdleMode(IdleMode mode) {
    m_frontLeftModule.getDriveMotor().setIdleMode(mode);
    m_frontRightModule.getDriveMotor().setIdleMode(mode);
    m_backLeftModule.getDriveMotor().setIdleMode(mode);
    m_backRightModule.getDriveMotor().setIdleMode(mode);
  }

  /**
   * Prints the estimated gyro value to the simulator.
   * 
   * @param printHeading The estimated gyro value.
   */
  public void printSimulatedGyro(double printHeading) {
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(printHeading);
  }

  /** Zeroes the odometry. */
  public void resetOdometry() {
    m_odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose  The pose to which to set the odometry.
   * @param angle The angle to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose, Rotation2d angle) {
    m_odometry.resetPosition(pose, angle);
  }
}
