// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    DataLogManager.start();
    SignalLogger.start();
    URCL.start();
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.setLEDs();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SmartDashboard.putString("AutoAlliance", DriverStation.getAlliance().toString());

    m_robotContainer.intakeCamera.setPipeline(1);
    stopAll();

    // Allows the simulation sensor to work on the proper side of the field
    m_robotContainer.colorReceived(DriverStation.getAlliance().orElse(Alliance.Blue));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    SmartDashboard.putString("TeleopAlliance", DriverStation.getAlliance().toString());

    m_robotContainer.intakeCamera.setPipeline(1);
    stopAll();
    m_robotContainer.createIntakeTrigger();

    // Allows the simulation sensor to work on the proper side of the field
    m_robotContainer.colorReceived(DriverStation.getAlliance().orElse(Alliance.Blue));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  private void stopAll() {
    m_robotContainer.shooter.setOffSpeed();
    m_robotContainer.arm.setCurrentPosition();
    m_robotContainer.intake.intakeOff();
  }
}
