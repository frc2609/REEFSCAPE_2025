// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.hardware.Pigeon2;

<<<<<<< HEAD
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
=======
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Timer;
>>>>>>> main
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ZeroPCM;
import frc.robot.commands.climber.MoveClimberToPosition;
<<<<<<< HEAD
import frc.robot.generated.TunerConstants;
=======

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PathPlannerAlignmentCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.climber.MoveClimberToPositionJog;
import frc.robot.commands.climber.MoveClimberToPositionSDB;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeRoll;
>>>>>>> main
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Telemetry;

public class RobotContainer {
<<<<<<< HEAD
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
=======
>>>>>>> main

  // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // // kSpeedAt12Volts desired top speed
  // private double MaxAngularRate =
  // RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
  // second
  // // max angular velocity

  // /* Setting up bindings for necessary control of the swerve drive platform */
  // private final SwerveRequest.FieldCentric drive = new
  // SwerveRequest.FieldCentric()
  // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) //
  // Add a 10% deadband
  // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop
  // control for drive motors
  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  // // private final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();
  // private final SwerveRequest.RobotCentric forwardStraight = new
  // SwerveRequest.RobotCentric()
  // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // private final Telemetry logger = new Telemetry(MaxSpeed);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second //
                                                                                    // max angular velocity

  private SendableChooser<Command> autoChooser;
  
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
<<<<<<< HEAD
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Limelight seaweed = new Limelight("limelight-seaweed");
    private final Pigeon2 pidgey = new Pigeon2(0, "CANivore");

=======
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
  
    // public final CommandSwerveDrivetrain drivetrain =
    // TunerConstants.createDrivetrain();
  
    // public final Limelight seaweed = new Limelight("limelight-seaweed");
    // private final Pigeon2 pidgey = new Pigeon2(0, "CANivore"); // Pigeon is on
    // roboRIO CAN Bus with device ID 0
    // // private final ResetGyro resetGyro = new ResetGyro(drivetrain, seaweed,
    // pidgey)
>>>>>>> main
    // /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
<<<<<<< HEAD
    private final Intake intake = new Intake(); 

    private double targetPosition = 0.1;

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        pidgey.clearStickyFault_BootDuringEnable();
        configureDrivetrainBindings();
        configureBindings();


=======
    private final Intake intake = new Intake();
    public static InstantCommand instantCommand = new InstantCommand();
  
    private double targetPosition = 0.0;
    private final Pigeon2 pidgey = new Pigeon2(0, "CANivore"); // Pigeon is on roboRIO CAN Bus with device ID 0
    private final String limeLightName = "limelight-seaweed";
    private Field2d m_field = new Field2d();
  
    private static double REEF_SIDE = 0.813;
  
    public final Limelight seaweed = new Limelight("limelight-seaweed");
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
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
>>>>>>> main
    }
  
    private void configureBindings() {
       
  joystick.a()
  .whileTrue(
      new InstantCommand(() -> intake.setSpeed(0.1))
  
      ).onFalse(
          new InstantCommand(() -> intake.stop())
      );
      
      joystick.b()
      .whileTrue(
          new InstantCommand(() -> intake.setSpeed(-0.1))
      
          ).onFalse(
              new InstantCommand(() -> intake.stop())
          ); 
  
  
          
  
         joystick.povUp()
          .onTrue(
              new MoveClimberToPosition(climber, -0.37)
          );
  
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
      autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName));
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

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
      
    double ID = 18;
    double distanceOffset = 1.5;
    double coralOffset = REEF_SIDE * -1/2;


<<<<<<< HEAD
        

        joystick.x()
        .onTrue(
            new MoveClimberToPosition(climber, 300.0)
        );

        joystick.a()
        .onTrue(
            new MoveClimberToPosition(climber, 0.0)
        );

        joystick.b()
        .onTrue(
            new ZeroPCM(climber)
        );
=======
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
>>>>>>> main


    joystick.x().onTrue(swerve.getPathPlannerCommandToAprilTag(new Pose2d(
      fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*distanceOffset,
      fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*distanceOffset,
      new Rotation2d(fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getRotation().getRadians() - Math.PI)
    )));


<<<<<<< HEAD
    private void configureDrivetrainBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));
                

        joystick.rightBumper().and(joystick.a().whileTrue(drivetrain.applyRequest(() -> brake))
        );
=======
    joystick.y().onTrue(new ResetGyro(swerve, seaweed, pidgey).withTimeout(0.75).andThen(new AlignCommand(swerve, seaweed, pidgey)).withTimeout(5));
>>>>>>> main

        joystick.rightBumper().and(joystick.b()
            .whileTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() ->
                        drivetrain.applyRequest(() -> facingAngle.withTargetDirection(Rotation2d.fromDegrees(45)))
                    ).withTimeout(0.1),
                    drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-joystick.getLeftY() * MaxSpeed).withVelocityY(-joystick.getLeftX() * MaxSpeed))
                )
            )
        );

        joystick.rightBumper().and(joystick.povUp()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)))
        );
        joystick.rightBumper().and(joystick.povUpRight()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(-.5)))
        );
        joystick.rightBumper().and(joystick.povUpLeft()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(.5)))
        );

        joystick.rightBumper().and(joystick.povDown()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)))
        );
        joystick.rightBumper().and(joystick.povDownRight()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(-.5)))
        );
        joystick.rightBumper().and(joystick.povDownLeft()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(.5)))
        );


        joystick.rightBumper().and(joystick.povRight()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)))
        );
        joystick.rightBumper().and(joystick.povLeft()
            .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)))
        );

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    //     drivetrain.registerTelemetry(logger::telemeterize);
    //}

    }
    public void robotInit() {
        for (int port = 5800; port <= 5810; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
  

    private void adjustTargetPosition(double delta) {
        this.targetPosition += delta;
        SmartDashboard.putNumber("Target Position", targetPosition);
<<<<<<< HEAD
=======
        
        // If the motor is currently moving, update the target immediately
        // if (joystick.rightBumper().getAsBoolean()) {
        //     climber.positionControlledMotor.goToPosition(this.targetPosition);
        // }
>>>>>>> main
    }
}
  
