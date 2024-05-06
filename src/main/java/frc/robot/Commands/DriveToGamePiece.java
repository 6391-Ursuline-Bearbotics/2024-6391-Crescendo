// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Vision.Detector;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class DriveToGamePiece extends Command {

  private Detector ll;
  private CommandSwerveDrivetrain drivetrain;
  private PIDController thetaController = new PIDController(2.0, 0, 0.05);
  public DriveToGamePiece(CommandSwerveDrivetrain drivetrain, Detector ll) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.ll = ll;
  }
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    //.withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.01).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double thetaOutput = 0;
  private double xOutput = 0.2; // Minimum speed to drive towards note
  private final double yOutput = 0;
  private double setpoint = 0;
  private Timer blindTimer = new Timer();
  private double vert = 0.0;
  private double xScaled = 0.0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(Units.degreesToRadians(4));
    blindTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		if (ll.hasTarget()){ // || distanceTimer.hasElapsed(convertVertToTime(ll.getNoteVertical()))
      vert = ll.getNoteVertical();
      blindTimer.reset();

      // Setpoint is the current robot rotation with additional robot relative LL reading
      setpoint = Math.toRadians(-ll.getNoteHorizontal())+ drivetrain.getState().Pose.getRotation().getRadians();
      SmartDashboard.putNumber("Game Piece setpoint", setpoint);
			thetaController.setSetpoint(setpoint);
      thetaOutput = thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), setpoint);
      SmartDashboard.putNumber("theta output", thetaOutput);

      xScaled = xOutput + Math.max((vert + 14) * 0.02, 0); // speed scale
		} else {
      // Normally stop unless we just put down the intake then drive for 1.5 second, drive straight
      thetaOutput = 0;
      if (blindTimer.hasElapsed(1.5)) {
        xScaled = 0;
      } else {
        xScaled = xOutput;
      }
    }

    SmartDashboard.putNumber("notescaled", xScaled);
    drivetrain.setControl(drive.withVelocityX(xScaled * TunerConstants.kSpeedAt12VoltsMps).withVelocityY(yOutput).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}