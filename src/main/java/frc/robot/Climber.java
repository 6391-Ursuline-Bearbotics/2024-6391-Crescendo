// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX m_climbMotor = new TalonFX(6);
  private final VoltageOut m_climbRequest = new VoltageOut(0);
  private final double climbSpeed = 6; // This is in voltage out of 12

  /** Creates a new Climber. */
  public Climber() {
    TalonFXConfiguration climbTalonConfig = new TalonFXConfiguration();
    climbTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbTalonConfig.Voltage.PeakReverseVoltage = 0; // Never go in reverse
    m_climbMotor.setInverted(false);
    m_climbMotor.optimizeBusUtilization();
    m_climbMotor.getConfigurator().apply(climbTalonConfig);
  }

  public Command climb() {
    return climbVoltage(climbSpeed)
        .andThen(waitSeconds(2))
        .andThen(climbVoltage(0));
  }

  public Command climbVoltage(double voltage) {
    return runOnce(() -> m_climbMotor.setControl(m_climbRequest.withOutput(voltage)));
  }
}
