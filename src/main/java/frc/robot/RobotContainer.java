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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private final double TurtleAngularRate = Math.PI * 0.5; // .25 rotation per second max angular velocity.  Adjust for max turning rate speed.
  private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxPS5Controller drv = new CommandXboxPS5Controller(0); // driver xbox controller
  CommandXboxPS5Controller op = new CommandXboxPS5Controller(1); // operator xbox controller
  CommandSwerveDrivetrain drivetrain;
  RoboticPathing robo;
  Command topRoboticRoutine;
  Command botRoboticRoutine;

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

  // Starting the other subsystems
  private final LEDSubsystem lights = new LEDSubsystem();
  public final Detector intakeCamera = new Detector("limelight-note");
  private final Limelight shooterCamera;
  public final Arm arm = new Arm();
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  
  // Field-centric driving in Open Loop, can change to closed loop after characterization 
  // For closed loop replace DriveRequestType.OpenLoopVoltage with DriveRequestType.Velocity
  DriveRequestType controlType = DriveRequestType.OpenLoopVoltage;
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(controlType)
      .withDeadband(0) // Deadband is handled on input
      .withRotationalDeadband(0);

  SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(controlType);

  SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(controlType);

  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  Telemetry logger = new Telemetry(MaxSpeed);

  private Supplier<SwerveRequest> controlStyle;

  private String lastControl = "2 Joysticks";
  private Double lastSpeed = 0.65;

  private Translation2d speaker;
  private boolean blue = false;
  private boolean turtle = false;
  private ShotParameter shot;

  private void configureBindings() {
    newControlStyle();
    newSpeed();

    // Set up Driver Controls ================================================
    // Does full automation while held on the Amp side of the field
    drv.x().onFalse(runOnce(() -> SmartDashboard.putBoolean("autoControlled", false)));

    // Does full automation while held though the middle of the field
    drv.y().whileTrue(runOnce(() -> SmartDashboard.putBoolean("autoControlled", true)).andThen(either(
      either(robo.midRobotic, robo.MidAmpRobotic, () -> SmartDashboard.getBoolean("speaker",false)),
      robo.midSourceRobotic,
      () -> SmartDashboard.getBoolean("noteLoaded", false)).repeatedly()));
    drv.y().onFalse(runOnce(() -> SmartDashboard.putBoolean("autoControlled", false)));

    // Does full automation while held on the Source side of the field
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

    // Turtle Mode toggle
    drv.leftBumper().onTrue(either(
        runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * TurtleSpeed)
            .andThen(() -> AngularRate = TurtleAngularRate)
            .alongWith(runOnce(() -> turtle = false)),
        runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * speedChooser.getSelected())
            .andThen(() -> AngularRate = MaxAngularRate)
            .alongWith(runOnce(() -> turtle = true)),
        () -> turtle));

    drv.rightBumper().whileTrue(drivetrain.run(() -> robotCentered()));

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

    // Store the arm just inside the frame perimeter
    op.b().onTrue(arm.setStorePosition()
        .alongWith(shooter.setOffSpeed())
        .alongWith(intake.intakeOff()));

    // Get ready to intake the note
    op.a().onTrue(arm.setIntakePosition()
        .alongWith(shooter.setOffSpeed())
        .alongWith(intake.intakeOn()));

    // Shoot from the Wing Line
    op.povUp().onTrue(arm.setWingShootPosition()
        .alongWith(shooter.setWingSpeed())
        .alongWith(intake.intakeOff()));    

    // Shoot from the Auto Circle
    op.povLeft().onTrue(arm.setAutoShootPosition()
        .alongWith(shooter.setAutoSpeed())
        .alongWith(intake.intakeOff()));

    // Shoot from the Subwoofer
    op.povDown().onTrue(arm.setSubShootPosition()
        .alongWith(shooter.setSubSpeed())
        .alongWith(intake.intakeOff()));

    // Shoot from Interpolated Spot
    op.povRight().whileTrue(distanceShot(() -> getSpeakerDistance()).repeatedly()
        .alongWith(intake.intakeOff()));

    // Controls if the Robotic Pathing will drive to the speaker or amp
    op.leftBumper().onTrue(runOnce(() -> SmartDashboard.putBoolean("speaker", true)));
    op.rightBumper().onTrue(runOnce(() -> SmartDashboard.putBoolean("speaker", false)));

    // Change angle manually
    op.leftTrigger().onTrue(arm.relativeAngleChange(-1.0));
    op.rightTrigger().onTrue(arm.relativeAngleChange(1.0));

    op.back().onTrue(shooter.relativeSpeedChange(-10.0));
    op.start().onTrue(shooter.relativeSpeedChange(10.0));

    // TRIGGERS==================================================================
    // When a note is detected by the camera near the intake turn the lights green
    Trigger noteTrigger = new Trigger(() -> intakeCamera.hasTarget());
    noteTrigger.onTrue(runOnce(() -> SmartDashboard.putBoolean("noteFound", true)));
    noteTrigger.onFalse(runOnce(() -> SmartDashboard.putBoolean("noteFound", false)));

    Trigger controlPick = new Trigger(() -> lastControl != controlChooser.getSelected());
    controlPick.onTrue(runOnce(() -> newControlStyle()));

    Trigger speedPick = new Trigger(() -> lastSpeed != speedChooser.getSelected());
    speedPick.onTrue(runOnce(() -> newSpeed()));

    // If all conditions met light LED green (may later auto shoot)
    Trigger readyToShoot = new Trigger(
        () -> shooter.atSetpoint()).and(
        () -> SmartDashboard.getBoolean("noteLoaded", false));
    readyToShoot.onTrue(runOnce(() -> SmartDashboard.putBoolean("readyToShoot", true)));
    readyToShoot.onFalse(runOnce(() -> SmartDashboard.putBoolean("readyToShoot", false)));

/*     // All of these bindings are for System Indentification and will be disabled at competition
    drv.leftTrigger().and(drv.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    drv.leftTrigger().and(drv.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    drv.leftTrigger().and(drv.pov(270)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    drv.leftTrigger().and(drv.pov(90)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    drv.rightTrigger().and(drv.pov(0)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    drv.rightTrigger().and(drv.pov(180)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    drv.rightTrigger().and(drv.pov(270)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    drv.rightTrigger().and(drv.pov(90)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));

    op.back().whileTrue(arm.dynamicForward());
    op.start().whileTrue(arm.dynamicBackward());
    op.leftTrigger().whileTrue(arm.quasistaticForward());
    op.rightTrigger().whileTrue(arm.quasistaticBackward()); */

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
    NamedCommands.registerCommand("armAutoShootPosition", arm.setAutoShootPosition());
    NamedCommands.registerCommand("armSubShootPosition", arm.setSubShootPosition());
    NamedCommands.registerCommand("armAmpPosition", arm.setAmpPosition());
    NamedCommands.registerCommand("intakeOn", intake.intakeAutoStop());

    drivetrain = TunerConstants.DriveTrain; // Make Drivetrain after Named Commands
    robo = new RoboticPathing();
    shooterCamera = new Limelight(drivetrain, "limelight-tag");

    autoAim.HeadingController.setPID(3.0, 0.0, 0.5);
    autoAim.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    //PPHolonomicDriveController.setRotationTargetOverride(Optional.of(getSpeakerRotation()));

    // Creating the routine commands here for top/bot so we can switch them depending on Alliance
    topRoboticRoutine = runOnce(() -> SmartDashboard.putBoolean("autoControlled", true)).andThen(either(
        either(robo.topRobotic, robo.TopAmpRobotic, () -> SmartDashboard.getBoolean("speaker",false)),
        robo.topSourceRobotic,
        () -> SmartDashboard.getBoolean("noteLoaded", false)).repeatedly());

    botRoboticRoutine = runOnce(() -> SmartDashboard.putBoolean("autoControlled", true)).andThen(either(
        either(robo.botRobotic, robo.BotAmpRobotic, () -> SmartDashboard.getBoolean("speaker",false)),
        robo.botSourceRobotic,
        () -> SmartDashboard.getBoolean("noteLoaded", false)).repeatedly());

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
        controlStyle = () -> drive.withVelocityX(conditionX(-drv.getLeftY()) * MaxSpeed) // Drive forward -Y
            .withVelocityY(conditionY(-drv.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(conditionRot(-drv.getRightX()) * AngularRate); // Drive counterclockwise with negative X (left)
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

  private double conditionX(double joystick) {
    return xLimiter.calculate(MathUtil.applyDeadband(joystick, 0.05));
  }

  private double conditionY(double joystick) {
    return yLimiter.calculate(MathUtil.applyDeadband(joystick, 0.05));
  }

  private double conditionRot(double joystick) {
    return rotLimiter.calculate(MathUtil.applyDeadband(joystick, 0.05));
  }

  private Command distanceShot(Supplier<Double> distance) {
    return runOnce(() -> shot = InterpolatingTable.get(distance.get()))
        .andThen(shooter.runOnce(() -> shooter.setRPS(shot.rps))
            .alongWith(arm.runOnce(() -> arm.setGoal(shot.angle))));
  }

  public void colorReceived(Alliance ally) {
    if (ally == Alliance.Blue) {
      blue = true;
      drv.x().whileTrue(topRoboticRoutine);
      drv.b().whileTrue(botRoboticRoutine);
    } else {
      blue = false;
      drv.x().whileTrue(botRoboticRoutine);
      drv.b().whileTrue(topRoboticRoutine);
    }
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

  private Rotation2d getSpeakerRotation() {
    if (!blue) {
      speaker = Constants.Field.redSpeaker;
    } else {
      speaker = Constants.Field.blueSpeaker; //getMovingSpeaker(true);
    }
    Rotation2d currentAngle = drivetrain.getState().Pose.getRotation().plus(new Rotation2d(Math.PI));
    Rotation2d setpoint = speaker.minus(drivetrain.getState().Pose.getTranslation()).getAngle();
    SmartDashboard.putNumber("Auto Aim Robot", currentAngle.getRadians());
    SmartDashboard.putNumber("Auto Aim Setpoint", setpoint.getRadians());
    return setpoint;
  }

  private Double getSpeakerDistance() {
    if (!blue) {
      SmartDashboard.putNumber("SpeakerDistance", drivetrain.getState().Pose.getTranslation().getDistance(Constants.Field.redSpeaker));
      return drivetrain.getState().Pose.getTranslation().getDistance(Constants.Field.redSpeaker);
    } else {
      SmartDashboard.putNumber("SpeakerDistance", drivetrain.getState().Pose.getTranslation().getDistance(Constants.Field.blueSpeaker));
      return drivetrain.getState().Pose.getTranslation().getDistance(Constants.Field.blueSpeaker);
    }
  }

  private void autoAim() {
    Rotation2d speaker = getSpeakerRotation();
    // The operator perspective stuff inverts everything field oriented
    if (blue) {
      speaker = speaker.plus(new Rotation2d(Math.PI));
    }
    drivetrain.setControl(autoAim.withVelocityX(-drv.getLeftY() * MaxSpeed).withVelocityY(-drv.getLeftX() * MaxSpeed).withTargetDirection(speaker));
  }

  private void robotCentered() {
    drivetrain.setControl(robotDrive.withVelocityX(-drv.getLeftY() * MaxSpeed).withVelocityY(-drv.getLeftX() * MaxSpeed).withRotationalRate(conditionRot(-drv.getRightX()) * AngularRate));
  }

  public Translation2d getMovingSpeaker(boolean blue) {
    Translation2d goalPose = blue ? Constants.Field.blueSpeaker : Constants.Field.redSpeaker;
    ChassisSpeeds robotVel = drivetrain.getCurrentRobotChassisSpeeds();
    double distanceToSpeaker = drivetrain.getState().Pose.getTranslation().getDistance(goalPose);
    double x = goalPose.getX() - (robotVel.vxMetersPerSecond * (distanceToSpeaker / Constants.Field.noteVelocity));
    double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distanceToSpeaker / Constants.Field.noteVelocity));
    return new Translation2d(x, y);
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

  public void createIntakeTrigger() {
    // Turn the intake off whenever the note gets to the sensor
    intake.getIntakeStopSensor().onTrue(intake.intakeOff()
        .alongWith(arm.setStorePosition())
        .alongWith(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", true)))
        .alongWith(intakeCamera.blinkLEDS())
        .alongWith(shooterCamera.blinkLEDS()));
    intake.getIntakeStopSensor().onFalse(
        new WaitCommand(0.2).andThen(shooter.setOffSpeed())
        .alongWith(runOnce(() -> SmartDashboard.putBoolean("noteLoaded", false))));
  }
}
