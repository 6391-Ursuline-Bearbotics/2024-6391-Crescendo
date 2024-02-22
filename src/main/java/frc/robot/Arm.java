// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static final int armPrimaryID = 2;
  private static final int armFollowerID = 1;
  private CANSparkMax m_motor;
  private CANSparkMax m_follower;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_relativeEncoder;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private ArmFeedforward m_armFF;
  private Boolean disabled = false;

  // Arm setpoints in  rotations
  private static final double intakePosition = 0.0;
  private static final double shootPosition = 0.08;
  private static final double storePosition = 0.15;
  private static final double ampPosition = 0.25;

  // Arm Contraints
  private static final double kMaxVelocityRadPerSecond = Math.PI / 2; // 90deg per second
  private static final double kMaxAccelerationRadPerSecSquared = Math.PI;
  // The value (inverted) when measured parallel to the ground making it 0
  private static final double kArmOffsetRads = 0.332;

  // Profile Setup
  private final TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_state;
  private TrapezoidProfile.State m_goal;

  /** Creates a new Arm. */
  public Arm() {
    // initialize 2 NEO in follower setup
    m_motor = new CANSparkMax(armPrimaryID, MotorType.kBrushless);
    m_follower = new CANSparkMax(armFollowerID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_follower.restoreFactoryDefaults();
    m_motor.setInverted(true);
    m_follower.setInverted(false);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_follower.setIdleMode(IdleMode.kBrake);
    m_follower.follow(m_motor);

    // Get integrated NEO encoder
    m_relativeEncoder = m_motor.getEncoder();
    // REV Throughbore encoder hooked to SparkMAX using the Absolute Encoder Adapter
    m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    //m_absoluteEncoder.setPositionConversionFactor((2 * Math.PI));
    m_absoluteEncoder.setInverted(false);
    m_absoluteEncoder.setZeroOffset(kArmOffsetRads);

    // Setting up the onboard PID controller on the SparkMAX
    m_pidController = m_motor.getPIDController();
    m_pidController.setP(0.1);
    m_pidController.setI(0);
    m_pidController.setD(0);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-0.2, 0.2);
    m_pidController.setFeedbackDevice(m_absoluteEncoder);

    m_motor.burnFlash();

    m_armFF = new ArmFeedforward(0, 0.96, 0);

    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared));
    double initialPosition = m_absoluteEncoder.getPosition();
    m_state = new TrapezoidProfile.State(initialPosition, 0);
    m_goal = new TrapezoidProfile.State(initialPosition, 0);
  }

  @Override
  public void periodic() {
    // Stop the arm if the relative encoder has velocity but the absolute doesn't
    if ((m_relativeEncoder.getVelocity() > 1 && m_absoluteEncoder.getVelocity() < 1)
        || (Math.abs(m_motor.getOutputCurrent() - m_follower.getOutputCurrent()) > 50)) {
      disabled = true;
    }

    if (!disabled) {
      // Update the Trapezoid profile
      m_state = m_profile.calculate(0.02, m_goal, m_state);
      // Calculate the "feedforward" from the current angle turning it into a form of feedback
      double position = m_absoluteEncoder.getPosition();
      SmartDashboard.putNumber("Arm Position", position);
      double feedforward = m_armFF.calculate(position * 2 * Math.PI, m_state.velocity);
      // Add the feedforward to the PID output to get the motor output
      m_pidController.setReference(m_state.position, ControlType.kPosition, 0, feedforward);
    }
  }

  public Command setIntakePosition() {
    return setArmGoalCommand(intakePosition);
  }

  public Command setShootPosition() {
    return setArmGoalCommand(shootPosition);
  }

  public Command setAmpPosition() {
    return setArmGoalCommand(ampPosition);
  }

  public Command setStorePosition() {
    return setArmGoalCommand(storePosition);
  }

  public Command setArmGoalCommand(double goal) {
    return Commands.runOnce(() -> setGoal(goal), this);
  }

  /**
   * Sets the goal state for the subsystem. Goal velocity assumed to be zero.
   *
   * @param goal The goal position for the subsystem's motion profile.
   */
  public final void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0);
  }
}
