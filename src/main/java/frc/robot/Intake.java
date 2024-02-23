// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
  private static final int intakePrimaryID = 5;
  private static final int intakeSensorPort = 0;
  private CANSparkMax m_motor;

  private static final double intakeSpeed = 0.6;
  private DigitalInput noteSensor = new DigitalInput(intakeSensorPort);

  /** Creates a new Arm. */
  public Intake() {
    // initialize motor
    m_motor = new CANSparkMax(intakePrimaryID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(true);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.enableVoltageCompensation(12);
    m_motor.burnFlash();
  }

  public Command intakeOn() {
    return this.runOnce(() -> m_motor.setVoltage(intakeSpeed * 12));
  }

  public Command intakeOff() {
    return this.runOnce(() -> m_motor.setVoltage(0));
  }

  public Command shoot() {
    return this.run(() -> m_motor.setVoltage(intakeSpeed * 12)).withTimeout(0.2)
        .andThen(this.runOnce(() -> m_motor.set(0)));
  }

  public Trigger getIntakeSensor() {
    return new Trigger(() -> !noteSensor.get());
  }
}
