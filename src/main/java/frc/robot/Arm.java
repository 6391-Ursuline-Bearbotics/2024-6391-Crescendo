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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Util.InterpolatingTable;

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
  private static final double intakePosition = 0.00; // -3.9
  private static final double storePosition = 18;
  private static final double climbPosition = 60.0;
  private static final double ampPosition = 92.0;

  // Arm Contraints
  private static final double kMaxVelocityRadPerSecond = Math.PI / 3; // 90deg per second
  private static final double kMaxAccelerationRadPerSecSquared = Math.PI / 2  ;
  // The value (inverted) when measured parallel to the ground making it 0
  private static final double kArmOffsetRads = 0.022; //  .028

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
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5,20); // set status 5 periodic to 20ms
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
    m_pidController.setP(.68); // 2
    m_pidController.setI(0.000);
    m_pidController.setD(0); //.5
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
//    m_pidController.setOutputRange(-0.12, 0.22); //allowed output of arm
    m_pidController.setFeedbackDevice(m_absoluteEncoder);

    m_motor.burnFlash();
    m_follower.burnFlash();

    sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> m_motor.setVoltage(voltage.in(Volts)),
                        null, // No log consumer, since data is recorded by URCL
                        this));

    m_armFF = new ArmFeedforward(.45, .55, 2.5); // kg .47 kv 4

    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared));
    double initialPosition = removeWrap(m_absoluteEncoder.getPosition(), storePosition - 2);
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
      m_state = m_profile.calculate(0.02, m_state, m_goal);
      // Calculate the "feedforward" from the current angle turning it into a form of feedback
      position = removeWrap(m_absoluteEncoder.getPosition(), 0);
      SmartDashboard.putNumber("Arm Position", position * 120.0);
      SmartDashboard.putNumber("Arm Setpoint", m_state.position * 120.0);
      SmartDashboard.putNumber("Arm Current", m_motor.getOutputCurrent());
      double feedforward = m_armFF.calculate(position * 2 * Math.PI / 3, m_state.velocity);
      // Add the feedforward to the PID output to get the motor output
      m_pidController.setReference(m_state.position, ControlType.kPosition, 0, feedforward);
    }
  }

//
// We should set dead zones around expected ranges. if anything over 98, or anything below 17
// we will never have a hold at this setpoint between auto / tele and these would be out-of bounds 
//
  private double removeWrap(double value, double deadzone) {
    if (value > 0.9 || value < deadzone) { // make sure we do not get a non-zero value in the initial setpoint
      return 0.0;
    } else {
      return value;
    }
  }

  public Command setCurrentPosition() {
    return setArmGoalCommand(position * 120.0);
  }

  public Command setIntakePosition() {
    return setArmGoalCommand(intakePosition);
  }

  public Command setWingShootPosition() {
    return setArmGoalCommand(InterpolatingTable.wing.angle);
  }

  public Command setFarWingShootPosition() {
    return setArmGoalCommand(InterpolatingTable.farwing.angle);
  }

  public Command setStageShootPosition() {
    return setArmGoalCommand(InterpolatingTable.stage.angle);
  }

  public Command setAutoShootPosition() {
    return setArmGoalCommand(InterpolatingTable.auto.angle);
  }

  public Command setSubShootPosition() {
    return setArmGoalCommand(InterpolatingTable.sub.angle);
  }

  public Command setClimbPosition() {
    return setArmGoalCommand(climbPosition);
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
    return new InstantCommand(() -> setGoal(goal));
  }

  /**
   * Sets the goal state for the arm. Goal velocity assumed to be zero.
   *
   * @param goal The goal position for the arm's motion profile.  In degrees
   */
  public final void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(Math.min(goal, 100) / 120.0, 0);
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

  public Command relativeAngleChange(double degrees) {
    return runOnce(() -> setGoal((m_goal.position * 120) + degrees));
  }
}
