// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
=======
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.List;
>>>>>>> origin/Swerve_LimelightPathPlanner_DNW
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;

<<<<<<< HEAD
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.net.PortForwarder;
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
>>>>>>> origin/Swerve_LimelightPathPlanner_DNW
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.PathToAprilTagCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.climber.MoveClimberToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Telemetry;
=======
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.PathPlannerAlignmentCommand;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.AlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
>>>>>>> origin/Swerve_LimelightPathPlanner_DNW

public class RobotContainer {
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    //                                                                                   // max angular velocity

    // /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final Limelight seaweed = new Limelight("limelight-seaweed");
    // private final Pigeon2 pidgey = new Pigeon2(0, "CANivore"); // Pigeon is on roboRIO CAN Bus with device ID 0
    // // private final ResetGyro resetGyro = new ResetGyro(drivetrain, seaweed, pidgey);

    // /* Path follower */
    // private final SendableChooser<Command> autoChooser;
    // private final Arm arm = new Arm();
    // private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final Intake intake = new Intake(); 
public static InstantCommand instantCommand = new InstantCommand();



private double targetPosition = 0.0;

    public RobotContainer() {

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
        // pidgey.clearStickyFault_BootDuringEnable();

<<<<<<< HEAD
        configureBindings();
=======
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final Pigeon2 pidgey = new Pigeon2(0, "CANivore"); // Pigeon is on roboRIO CAN Bus with device ID 0
  private final String limeLightName = "limelight-seaweed";
  private Field2d m_field = new Field2d();

  private static double REEF_SIDE = 0.813;

    public final Limelight seaweed = new Limelight("limelight-seaweed");
>>>>>>> origin/Swerve_LimelightPathPlanner_DNW


<<<<<<< HEAD
    }

    private void configureBindings() {
     
joystick.a()
.whileTrue(
    new InstantCommand(() -> intake.setSpeed(0.1))

    ).onFalse(
        new InstantCommand(() -> intake.stop())
    );
=======
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
      joystick.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));
      joystick.rightBumper().onTrue(swerve.runOnce(() ->swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName))));

      swerve.registerTelemetry(logger::telemeterize);

      // Schedule `exampleMethodCommand` when the Xbox controller's B button is
      // pressed,
      // cancelling on release.

    //path finder bindings
>>>>>>> origin/Swerve_LimelightPathPlanner_DNW
    
    joystick.b()
    .whileTrue(
        new InstantCommand(() -> intake.setSpeed(-0.1))
    
        ).onFalse(
            new InstantCommand(() -> intake.stop())
        ); 


<<<<<<< HEAD
        

       joystick.povUp()
        .onTrue(
            new MoveClimberToPosition(climber, -0.37)
        );
=======
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

    double ID = LimelightHelpers.getFiducialID(limeLightName);
    double distanceOffset = 1.5;
    double coralOffset = REEF_SIDE * -1/2;


    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);


    joystick.x().onTrue(swerve.getPathPlannerCommandToAprilTag(new Pose2d(
      fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*distanceOffset,
      fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*distanceOffset,
      new Rotation2d(fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getRotation().getRadians() - Math.PI)
    )));


    joystick.y().onTrue(new ResetGyro(swerve, seaweed, pidgey).withTimeout(0.75).andThen(new AlignCommand(swerve, seaweed, pidgey)).withTimeout(5));
    //joystick.y().onTrue(new AlignCommand(swerve, seaweed, pidgey));

 

    //joystick.x().onTrue(new PathPlannerAlignmentCommand(swerve).withTimeout(5));
  }
>>>>>>> origin/Swerve_LimelightPathPlanner_DNW

        // joystick.povDown()
        // .onTrue(
        //     new InstantCommand(() -> {
        //         climber.goToPosition(0);
        //     } )
        // );

    joystick.povDown()
    .onTrue(
        new MoveClimberToPosition(climber, 0.0)
        
    );


          

    }

    // private void configureDrivetrainBindings() {
    //     // Note that X is defined as forward according to WPILib convention,
    //     // and Y is defined as to the left according to WPILib convention.
    //     drivetrain.setDefaultCommand(
    //             // Drivetrain will execute this command periodically
    //             drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                                // negative Y
    //                                                                                                // (forward)
    //                     .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
    //                                                                                 // negative X (left)
    //             ));
                

    //     joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //     // joystick.b().whileTrue(drivetrain.applyRequest(
    //     // () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
    //     // -joystick.getLeftX()))));

    //     joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //     joystick.pov(180)
    //             .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    //     // Run SysId routines when holding back/start and X/Y.
    //     // Note that each routine should be run exactly once in a single log.
    //     joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //     joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //     joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //     joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //     // reset the field-centric heading on left bumper press
    //     joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    //     drivetrain.registerTelemetry(logger::telemeterize);
    // }

    public void robotInit() {
        for (int port = 5800; port <= 5810; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return autoChooser.getSelected();
        return null;
    }
}
