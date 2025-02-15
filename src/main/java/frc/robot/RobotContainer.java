// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.List;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second                                                                                  // max angular velocity


  private final SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final Pigeon2 pidgey = new Pigeon2(0, "CANivore"); // Pigeon is on roboRIO CAN Bus with device ID 0
  private final String limeLightName = "limelight-seaweed";
  private Field2d m_field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    // Use event markers as triggers
    new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
    pidgey.clearStickyFault_BootDuringEnable();
    // Configure the trigger b indings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    swerve.setDefaultCommand(
      // Drivetrain will execute this command periodically
      swerve.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                         // negative Y
                                                                                         // (forward)
              .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystick.getRightX() * MaxAngularRate)) // Drive counterclockwise with
                                                                          // negative X (left)      
                                                                                                                           
      );
      joystick.a().whileTrue(swerve.applyRequest(() -> brake));
      joystick.b().whileTrue(swerve.applyRequest(
      () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
      -joystick.getLeftX()))));

      joystick.pov(0).whileTrue(swerve.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
      joystick.pov(180)
              .whileTrue(swerve.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
      // drivetrain.applyRequest(() ->
      // forwardStraight.withVelocityX(LimelightHelpers.getTX(null)).withVelocityY(0));

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      joystick.back().and(joystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
      joystick.back().and(joystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
      joystick.start().and(joystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
      joystick.start().and(joystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

      // reset the field-centric heading on left bumper press
      //.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      swerve.registerTelemetry(logger::telemeterize);

      // Schedule `exampleMethodCommand` when the Xbox controller's B button is
      // pressed,
      // cancelling on release.

    //path finder bindings
    

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
      new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));

    //joystick.x().onTrue(swerve.getPathPlannerCommandToAprilTag(new Pose2d(1, 5, new Rotation2d(180))));

    LimelightHelpers.SetFidcuial3DOffset(limeLightName, 1, 1, 1);
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    double ID = LimelightHelpers.getFiducialID(limeLightName);    

    joystick.x().onTrue(swerve.getPathPlannerCommandToAprilTag(new Pose2d(
        fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getX() ,
        fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getY(),
        fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().toRotation2d()
        )
        ).withTimeout(5)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
