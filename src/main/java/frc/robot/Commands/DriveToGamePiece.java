// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Arm;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Intake;
import frc.robot.Vision.Detector;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class DriveToGamePiece extends Command {

  private Detector ll;
  private CommandSwerveDrivetrain drivetrain;
  private Intake intake;
  private Arm arm;
  private PIDController thetaController = new PIDController(2.0, 0, 0.05);
  public DriveToGamePiece(CommandSwerveDrivetrain drivetrain, Detector ll, Intake intake, Arm arm) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.ll = ll;
    this.intake = intake;
    this.arm = arm;
  }
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    //.withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.01).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double thetaOutput = 0;
  private double xOutput = 0.2; // Speed to drive towards note will increase after testing
  private final double yOutput = 0;
  private double setpoint = 0;
  private double distance = 0;
  private Timer distanceTimer = new Timer();
  private double vert = 0.0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		if (ll.hasTarget()){ // || distanceTimer.hasElapsed(convertVertToTime(ll.getNoteVertical()))
      vert = ll.getNoteVertical();
      if (distance == 0) {
        distance = ll.getNoteVertical(); // This is the initial distance, we may want to do special cases here
      }
      setpoint = Math.toRadians(-ll.getNoteHorizontal())+ drivetrain.getState().Pose.getRotation().getRadians();
      SmartDashboard.putNumber("Game Piece setpoint", setpoint);
			thetaController.setSetpoint(setpoint);
      if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), setpoint);
        SmartDashboard.putNumber("theta output", thetaOutput);
			}
		}
    double xScaled = 0.2 + (vert + 15) * 0.04;
    SmartDashboard.putNumber("notescaled", xScaled);
    drivetrain.setControl(drive.withVelocityX(xScaled * TunerConstants.kSpeedAt12VoltsMps).withVelocityY(yOutput).withRotationalRate(thetaOutput));

    if (vert < -10) {
      arm.setIntakePosition().alongWith(intake.intakeOn()).schedule();
    }
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

  private Pose2d getNotePose(double distance, double yaw) {
    return new Pose2d(null, null, null);
  }
}