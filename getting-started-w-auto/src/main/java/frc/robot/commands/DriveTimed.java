// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class DriveTimed extends CommandBase {
  private final Field2d m_field;
  private final DifferentialDrive m_drive;
  private final double m_time_s;
  private final double m_speed;
  private final Timer m_timer = new Timer();

  /**
   * Creates a new DriveTimed.
   *
   * @param time_s The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveTimed(double time_s, double speed, DifferentialDrive drive) {
    m_time_s = time_s;
    m_speed = speed;
    m_drive = drive;
    //addRequirements(m_drive);
    // make a new field to provide trajectory feedback
    m_field = new Field2d();
  }

  @Override
  public void initialize() {
    //m_drive.resetEncoders();
    m_drive.arcadeDrive(m_speed, 0);
    m_timer.reset();
    m_timer.start();
    // make the expected trajectory
    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
              new Pose2d(m_time_s, m_time_s, Rotation2d.fromDegrees(0))),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
    m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  public void putDashboardData() {
    SmartDashboard.putData(m_field);
  }

  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
    double time_now = m_timer.get();
    m_field.getRobotObject().setPose(new Pose2d(time_now, time_now, Rotation2d.fromDegrees(0)));
    SmartDashboard.putData(m_field);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_time_s;
  }
}