// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.DriveToGamePiece;
import frc.robot.Commands.StrafeToGamePiece;
import frc.robot.LED.LEDSubsystem;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Util.InterpolatingTable;
import frc.robot.Util.RectanglePoseArea;
import frc.robot.Util.RoboticPathing;
import frc.robot.Util.ShotParameter;
import frc.robot.Vision.Detector;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;
  private SendableChooser<String> controlChooser = new SendableChooser<>();
  private SendableChooser<Double> speedChooser = new SendableChooser<>();
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Initial max is true top speed
  private final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
  private final double MaxAngularRate = Math.PI * 1.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
  private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
  private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxPS5Controller drv = new CommandXboxPS5Controller(0); // driver xbox controller
  CommandXboxPS5Controller op = new CommandXboxPS5Controller(1); // operator xbox controller
  CommandSwerveDrivetrain drivetrain;
  RoboticPathing robo;  

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);

  // Starting the other subsystems
  private final LEDSubsystem lights = new LEDSubsystem();
  private final Detector intakeCamera = new Detector("limelight-note");
  private final Limelight shooterCamera;
  private final Arm arm = new Arm();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  
  // Field-centric driving in Open Loop, can change to closed loop after characterization 
  // For closed loop replace DriveRequestType.OpenLoopVoltage with DriveRequestType.Velocity
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(MaxSpeed * 0.1) // Deadband is handled on input
      .withRotationalDeadband(AngularRate * 0.1);

  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  private Supplier<SwerveRequest> controlStyle;

  private String lastControl = "2 Joysticks";
  private Double lastSpeed = 0.65;

  private PIDController thetaController = new PIDController(4.0, 0, 0.05);
  private Translation2d speaker;
  private Double thetaOutput;

  private void configureBindings() {
    newControlStyle();
    newSpeed();

    // Set up Driver Controls ================================================
    // Does full automation while held on the Amp side of the field
    drv.x().whileTrue(runOnce(() -> SmartDashboard.putBoolean("autoControlled", true)).andThen(either(
      either(robo.topRobotic, robo.TopAmpRobotic, () -> SmartDashboard.getBoolean("speaker",false)),
      robo.topSourceRobotic,
      () -> SmartDashboard.getBoolean("noteLoaded", false)).repeatedly()));
    drv.x().onFalse(runOnce(() -> SmartDashboard.putBoolean("autoControlled", false)));

    // Does full automation while held though the middle of the field
    drv.y().whileTrue(runOnce(() -> SmartDashboard.putBoolean("autoControlled", true)).andThen(either(
      either(robo.midRobotic, robo.MidAmpRobotic, () -> SmartDashboard.getBoolean("speaker",false)),
      robo.midSourceRobotic,
      () -> SmartDashboard.getBoolean("noteLoaded", false)).repeatedly()));
    drv.y().onFalse(runOnce(() -> SmartDashboard.putBoolean("autoControlled", false)));

    // Does full automation while held on the Source side of the field
    drv.b().whileTrue(runOnce(() -> SmartDashboard.putBoolean("autoControlled", true)).andThen(either(
      either(robo.botRobotic, robo.BotAmpRobotic, () -> SmartDashboard.getBoolean("speaker",false)),
      robo.botSourceRobotic,
      () -> SmartDashboard.getBoolean("noteLoaded", false)).repeatedly()));
    drv.b().onFalse(runOnce(() -> SmartDashboard.putBoolean("autoControlled", false)));

    // While held will stay aimed at the speaker driver still has translation control but not rotation
    drv.a().whileTrue(drivetrain.run(() -> autoAim()));

    // Until we have a real robot / sensor this will simulate loading a note
    drv.povUp().onTrue(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", true)));
    drv.povDown().onTrue(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", false)));

    // Temporary should be handled by the operator
    drv.povRight().onTrue(runOnce(() -> SmartDashboard.putBoolean("speaker", true)));
    drv.povLeft().onTrue(runOnce(() -> SmartDashboard.putBoolean("speaker", false)));
    
    // reset the field-centric heading on start button press
    drv.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Drives to the game piece using turn, it will not strafe
    drv.back().whileTrue(new DriveToGamePiece(drivetrain, intakeCamera));

    // Turtle Mode while held
    drv.leftBumper().onTrue(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * TurtleSpeed)
        .andThen(() -> AngularRate = TurtleAngularRate));
    drv.leftBumper().onFalse(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * speedChooser.getSelected())
        .andThen(() -> AngularRate = MaxAngularRate));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Set up for the operator controls =========================================
    // Run the intake pushing the disk into the shooter for either the speaker or amp
    op.x().onTrue(intake.shoot());

    // Get ready to score in the Amp
    op.y().onTrue(arm.setAmpPosition()
        .alongWith(shooter.setAmpSpeed())
        .alongWith(intake.intakeOff()));

    // Get ready to score in the Speaker
    op.b().onTrue(arm.setShootPosition()
        .alongWith(shooter.setAutoSpeed())
        .alongWith(intake.intakeOff()));

    // Get ready to intake the note
    op.a().onTrue(arm.setIntakePosition()
        .alongWith(shooter.setOffSpeed())
        .alongWith(intake.intakeOn()));

    // TRIGGERS==================================================================
    // When a note is detected by the camera near the intake turn the lights green
    Trigger noteTrigger = new Trigger(() -> intakeCamera.hasTarget());
    noteTrigger.onTrue(runOnce(() -> SmartDashboard.putBoolean("noteFound", true)));
    noteTrigger.onFalse(runOnce(() -> SmartDashboard.putBoolean("noteFound", false)));

    Trigger controlPick = new Trigger(() -> lastControl != controlChooser.getSelected());
    controlPick.onTrue(runOnce(() -> newControlStyle()));

    Trigger speedPick = new Trigger(() -> lastSpeed != speedChooser.getSelected());
    speedPick.onTrue(runOnce(() -> newSpeed()));

    // Turn the intake off whenever the note gets to the sensor
    intake.getIntakeSensor().onTrue(intake.intakeOff()
        .alongWith(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", true))));
    intake.getIntakeSensor().onFalse(
        new WaitCommand(0.2).andThen(shooter.setOffSpeed())
        .alongWith(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", false))));

    // All of these bindings are for System Indentification and will be disabled at competition
    drv.x().and(drv.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    drv.x().and(drv.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    drv.y().and(drv.pov(0)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    drv.y().and(drv.pov(180)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    drv.a().and(drv.pov(0)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    drv.a().and(drv.pov(180)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    drv.b().and(drv.pov(0)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    drv.b().and(drv.pov(180)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));

    // Drivetrain needs to be placed against a sturdy wall and test stopped immediately upon wheel slip
    drv.back().and(drv.pov(0)).whileTrue(drivetrain.runDriveSlipTest());
  }

  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create PathPlanner Named Commands for use in Autos
    NamedCommands.registerCommand("shooterAutoSpeed", shooter.setAutoSpeed());
    NamedCommands.registerCommand("shooterAmpSpeed", shooter.setAmpSpeed());
    NamedCommands.registerCommand("shooterOffSpeed", shooter.setOffSpeed());
    NamedCommands.registerCommand("shoot", intake.shoot());
    NamedCommands.registerCommand("armIntakePosition", arm.setIntakePosition());
    NamedCommands.registerCommand("armShootPosition", arm.setShootPosition());
    NamedCommands.registerCommand("armAmpPosition", arm.setAmpPosition());
    NamedCommands.registerCommand("intakeOn", intake.intakeOn());
    NamedCommands.registerCommand("intakeOff", intake.intakeOff());

    drivetrain = TunerConstants.DriveTrain; // Make Drivetrain after Named Commands
    robo = new RoboticPathing();
    shooterCamera = new Limelight(drivetrain, "limelight");

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    controlChooser.setDefaultOption("2 Joysticks", "2 Joysticks");
    controlChooser.addOption("1 Joystick Rotation Triggers", "1 Joystick Rotation Triggers");
    controlChooser.addOption("Split Joysticks Rotation Triggers", "Split Joysticks Rotation Triggers");
    controlChooser.addOption("2 Joysticks with Gas Pedal", "2 Joysticks with Gas Pedal");
    SmartDashboard.putData("Control Chooser", controlChooser);

    speedChooser.addOption("100%", 1.0);
    speedChooser.addOption("95%", 0.95);
    speedChooser.addOption("90%", 0.9);
    speedChooser.addOption("85%", 0.85);
    speedChooser.addOption("80%", 0.8);
    speedChooser.addOption("75%", 0.75);
    speedChooser.addOption("70%", 0.7);
    speedChooser.setDefaultOption("65%", 0.65);
    speedChooser.addOption("60%", 0.6);
    speedChooser.addOption("55%", 0.55);
    speedChooser.addOption("50%", 0.5);
    speedChooser.addOption("35%", 0.35);
    SmartDashboard.putData("Speed Limit", speedChooser);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putBoolean("speaker", false);
    SmartDashboard.putBoolean("noteLoaded", false);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  private void newControlStyle() {
    lastControl = controlChooser.getSelected();
    switch (controlChooser.getSelected()) {
      case "2 Joysticks":
        controlStyle = () -> drive.withVelocityX(-drv.getLeftY() * MaxSpeed) // Drive forward -Y
            .withVelocityY(-drv.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-drv.getRightX() * AngularRate); // Drive counterclockwise with negative X (left)
        break;
      case "1 Joystick Rotation Triggers":
        controlStyle = () -> drive.withVelocityX(-drv.getLeftY() * MaxSpeed) // Drive forward -Y
            .withVelocityY(-drv.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate((drv.getLeftTriggerAxis() - drv.getRightTriggerAxis()) * AngularRate);
            // Left trigger turns left, right trigger turns right
        break;
      case "Split Joysticks Rotation Triggers":
        controlStyle = () -> drive.withVelocityX(-drv.getLeftY() * MaxSpeed) // Left stick forward/back
            .withVelocityY(-drv.getRightX() * MaxSpeed) // Right stick strafe
            .withRotationalRate((drv.getLeftTriggerAxis() - drv.getRightTriggerAxis()) * AngularRate);
            // Left trigger turns left, right trigger turns right
        break;
      case "2 Joysticks with Gas Pedal":
        controlStyle = () -> {
            var stickX = -drv.getLeftX();
            var stickY = -drv.getLeftY();
            var angle = Math.atan2(stickX, stickY);
            return drive.withVelocityX(Math.cos(angle) * drv.getRightTriggerAxis() * MaxSpeed) // left x * gas
                .withVelocityY(Math.sin(angle) * drv.getRightTriggerAxis() * MaxSpeed) // Angle of left stick Y * gas pedal
                .withRotationalRate(-drv.getRightX() * AngularRate); // Drive counterclockwise with negative X (left)
        };
        break;
    }
    try {
      drivetrain.getDefaultCommand().cancel();
    } catch(Exception e) {}
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(controlStyle).ignoringDisable(true));
  }

  private void newSpeed() {
    lastSpeed = speedChooser.getSelected();
    MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * lastSpeed;
  }

  private double conditionX(double joystick, double deadband) {
    return xLimiter.calculate(MathUtil.applyDeadband(joystick, deadband));
  }

  private Command distanceShot(double distance) {
    ShotParameter shot = InterpolatingTable.get(distance);
    return shooter.runOnce(() -> shooter.setRPS(shot.rpm / 60.0))
      .alongWith(arm.runOnce(() -> arm.setGoal(shot.angle)));
  }

  public void colorReceived(Alliance ally) {
    if (Utils.isSimulation()) {
      if (ally == Alliance.Blue) {
        // In sim when we go near the spearker / amp trigger a note release
        RectanglePoseArea shootArea = new RectanglePoseArea(
          new Translation2d(0.0, 2.75),
          new Translation2d(3.75, 8.27));
        Trigger shootTrigger = new Trigger(() -> shootArea.isPoseWithinArea(drivetrain.getState().Pose));
        shootTrigger.onTrue(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", false)));

        // In sim when we go near source trigger a note pickup
        RectanglePoseArea pickupArea = new RectanglePoseArea(
          new Translation2d(12.5, 0.0),
          new Translation2d(16.54, 3.75));
        Trigger pickupTrigger = new Trigger(() -> pickupArea.isPoseWithinArea(drivetrain.getState().Pose));
        pickupTrigger.onTrue(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", true)));
      } else {
        // In sim when we go near the spearker / amp trigger a note release
        RectanglePoseArea shootArea = new RectanglePoseArea(
          new Translation2d(12.79, 2.75),
          new Translation2d(16.54, 8.27));
        Trigger shootTrigger = new Trigger(() -> shootArea.isPoseWithinArea(drivetrain.getState().Pose));
        shootTrigger.onTrue(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", false)));

        // In sim when we go near source trigger a note pickup
        RectanglePoseArea pickupArea = new RectanglePoseArea(
          new Translation2d(0.0, 0.0),
          new Translation2d(4.0, 3.75));
        Trigger pickupTrigger = new Trigger(() -> pickupArea.isPoseWithinArea(drivetrain.getState().Pose));
        pickupTrigger.onTrue(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", true)));
      }
    }
  }

  private void autoAim() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      speaker = Constants.Field.redSpeaker;
    } else {
      speaker = Constants.Field.blueSpeaker;
    }
    double setpoint = speaker.minus(drivetrain.getState().Pose.getTranslation()).getAngle().getRadians();
    SmartDashboard.putNumber("Auto Aim Setpoint", setpoint);
    double temp = thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), setpoint);
    if (!thetaController.atSetpoint() ){
      thetaOutput = temp;
      SmartDashboard.putNumber("Auto Aim", thetaOutput);
    }
    drivetrain.setControl(drive.withVelocityX(-drv.getLeftY() * MaxSpeed).withVelocityY(-drv.getLeftX() * MaxSpeed).withRotationalRate(thetaOutput));
  }

  public void setLEDs() {
    if (DriverStation.isDisabled()) {
      lights.rainbow();
    } else {
      if (SmartDashboard.getBoolean("autoControlled", false)) {
        lights.setAllBlink(Color.kPurple, 1.0);
      } else {
        if (SmartDashboard.getBoolean("noteLoaded", false)) {
          if (SmartDashboard.getBoolean("readyToShoot", false)) {
            lights.setAll(Color.kGreen);
          } else {
            lights.setAll(Color.kRed);
          }
        } else {
          if (SmartDashboard.getBoolean("noteFound", false)) {
            lights.setAllBlink(Color.kGreen, 1.0);
          } else {
            lights.setAll(Color.kBlue);
          }
        }
      }
    }
  }
}
