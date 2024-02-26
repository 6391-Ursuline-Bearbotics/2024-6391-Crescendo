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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

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
  private Double position = 0.0;
  private SysIdRoutine sysIdRoutine;

  // Arm setpoints in degrees
  private static final double intakePosition = 0.0;
  private static final double subwooferPosition = 8.0;
  private static final double autoPosition = 28.0;
  private static final double wingPosition = 41.5;
  private static final double storePosition = 45.0;
  private static final double ampPosition = 90.0;

  // Arm Contraints
  private static final double kMaxVelocityRadPerSecond = Math.PI / 2; // 90deg per second
  private static final double kMaxAccelerationRadPerSecSquared = Math.PI;
  // The value (inverted) when measured parallel to the ground making it 0
  private static final double kArmOffsetRads = 0.668;

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
    //m_motor.setSmartCurrentLimit(50);
    //m_follower.setSmartCurrentLimit(50);
    m_motor.setInverted(true);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_follower.setIdleMode(IdleMode.kBrake);
    m_follower.follow(m_motor, true);

    // Get integrated NEO encoder
    m_relativeEncoder = m_motor.getEncoder();
    // REV Throughbore encoder hooked to SparkMAX using the Absolute Encoder Adapter
    m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_absoluteEncoder.setInverted(true);
    m_absoluteEncoder.setZeroOffset(kArmOffsetRads);

    // Setting up the onboard PID controller on the SparkMAX
    m_pidController = m_motor.getPIDController();
    m_pidController.setP(2);
    m_pidController.setI(0);
    m_pidController.setD(0);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-0.2, 0.2);
    m_pidController.setFeedbackDevice(m_absoluteEncoder);

    m_motor.burnFlash();
    m_follower.burnFlash();

    sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> m_motor.setVoltage(voltage.in(Volts)),
                        null, // No log consumer, since data is recorded by URCL
                        this));

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
/*     if ((m_relativeEncoder.getVelocity() > 1 && m_absoluteEncoder.getVelocity() < 1)
        || (Math.abs(m_motor.getOutputCurrent() - m_follower.getOutputCurrent()) > 50)) {
      disabled = true;
      m_motor.set(0);
    } */

    if (!disabled) {
      // Update the Trapezoid profile
      m_state = m_profile.calculate(0.02, m_goal, m_state);
      // Calculate the "feedforward" from the current angle turning it into a form of feedback
      position = m_absoluteEncoder.getPosition();
      SmartDashboard.putNumber("Arm Position", position * 120.0);
      SmartDashboard.putNumber("Arm Setpoint", m_state.position * 120.0);
      SmartDashboard.putNumber("Arm Current", m_motor.getOutputCurrent());
      double feedforward = m_armFF.calculate(position * 2 * Math.PI, m_state.velocity);
      // Add the feedforward to the PID output to get the motor output
      m_pidController.setReference(m_state.position, ControlType.kPosition, 0, feedforward);
    }
  }

  public Command setCurrentPosition() {
    return setArmGoalCommand(position);
  }

  public Command setIntakePosition() {
    return setArmGoalCommand(intakePosition);
  }

  public Command setWingShootPosition() {
    return setArmGoalCommand(wingPosition);
  }

  public Command setAutoShootPosition() {
    return setArmGoalCommand(autoPosition);
  }

  public Command setSubShootPosition() {
    return setArmGoalCommand(subwooferPosition);
  }

  public Command setAmpPosition() {
    return setArmGoalCommand(ampPosition);
  }

  public Command setStorePosition() {
    return setArmGoalCommand(storePosition);
  }

    /**
   * Gets a command that will set the position of the Arm.
   *
   * @param goal The goal position for the arm in degrees.
   * 
   * @return Command thats sets the trapezoid profile goal for the arm subsystem
   */
  public Command setArmGoalCommand(double goal) {
    return runOnce(() -> setGoal(goal));
  }

  /**
   * Sets the goal state for the arm. Goal velocity assumed to be zero.
   *
   * @param goal The goal position for the arm's motion profile.  In degrees
   */
  public final void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal / 120.0, 0);
  }

  public Command quasistaticForward() {
    return sysIdRoutine.quasistatic(Direction.kForward).until(() -> {
        return m_absoluteEncoder.getPosition() >= 0.7;
    });
  }

  public Command quasistaticBackward() {
      return sysIdRoutine.quasistatic(Direction.kReverse).until(() -> {
          return m_absoluteEncoder.getPosition() <= 0.05;
      });
  }

  public Command dynamicForward() {
      return sysIdRoutine.dynamic(Direction.kForward).until(() -> {
          return m_absoluteEncoder.getPosition() >= 0.7;
      });
  }

  public Command dynamicBackward() {
      return sysIdRoutine.dynamic(Direction.kReverse).until(() -> {
          return m_absoluteEncoder.getPosition() <= 0.05;
      });
  }
}
