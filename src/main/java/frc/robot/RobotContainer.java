// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.JogPCMCommand;
import frc.robot.commands.PathToAprilTagCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ZeroPCM;
import frc.robot.commands.Intake.DeployIntakeCommand;
import frc.robot.commands.Intake.RetractIntakeCommand;
import frc.robot.commands.Intake.RunRollCommand;
import frc.robot.commands.Intake.StopRollCommand;
import frc.robot.commands.arm.MoveArmToGrabAlgaeCommand;
import frc.robot.commands.arm.MoveArmToGrabCoralCommand;
import frc.robot.commands.arm.MoveArmToHumanLoadCommand;
import frc.robot.commands.arm.MoveArmToNetCommand;
import frc.robot.commands.arm.MoveArmToScoreCommand;
import frc.robot.commands.arm.MoveArmToScoreL1Command;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.commands.climber.DeployClimberCommand;
import frc.robot.commands.elevator.DeployElevatorHumanCommand;
import frc.robot.commands.elevator.DeployElevatorL1Command;
import frc.robot.commands.elevator.DeployElevatorL2Command;
import frc.robot.commands.elevator.DeployElevatorL3Command;
import frc.robot.commands.elevator.DeployElevatorL4Command;
import frc.robot.commands.gripper.GripCoral;
import frc.robot.commands.gripper.ReleaseCoral;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PathPlannerAlignmentCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.AlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeRoll;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Telemetry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class RobotContainer {

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
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // The robot's subsystems and commands are defined here...

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle();

    private static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);
  

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();



    // roboRIO CAN Bus with device ID 0
    // pidgey)
    // /* Path follower */
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final IntakeRoll intakeRoll = new IntakeRoll();
    private final IntakeFlop intakeFlop = new IntakeFlop();
    public static InstantCommand instantCommand = new InstantCommand();

    private double targetPosition = 0.0;
    private final Pigeon2 pidgey = new Pigeon2(0, "CANivore"); // Pigeon is on roboRIO CAN Bus with device ID 0
    private final String limeLightName = "limelight-seaweed";
    private Field2d m_field = new Field2d();

    private static double REEF_SIDE = 0.813;

    public final Limelight seaweed = new Limelight("limelight-seaweed");
    private Gripper gripper = new Gripper();
    
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                boolean jog = false;

            pidgey.clearStickyFault_BootDuringEnable();
            // Configure the trigger b indings
            if (jog == false){
                configureBindings();
                configureDrivetrainBindings();
            } else {
                configureJogBindings();
            }

            autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
            SmartDashboard.putData("Auto Mode", autoChooser);
            drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName));
        }

        private void configureJogBindings() {

                driverController.a().and(driverController.povUp().whileTrue(new JogPCMCommand(arm, 1)));
                driverController.a().and(driverController.povDown().whileTrue(new JogPCMCommand(arm, -1)));

                driverController.x().and(driverController.povUp().whileTrue(new JogPCMCommand(climber, 1)));
                driverController.x().and(driverController.povDown().whileTrue(new JogPCMCommand(climber, -1)));

                driverController.y().and(driverController.povUp().whileTrue(new JogPCMCommand(elevator, 1)));
                driverController.y().and(driverController.povDown().whileTrue(new JogPCMCommand(elevator, -1)));

                driverController.b().and(driverController.povUp().whileTrue(new JogPCMCommand(intakeFlop, 1)));
                driverController.b().and(driverController.povDown().whileTrue(new JogPCMCommand(intakeFlop, -1)));

        }
    
        private void configureBindings() {
    
            // driverController.a()
            //         .whileTrue(
            //                 new InstantCommand(() -> intakeRoll.setSpeed(0.1))
    
            //         ).onFalse(
            //                 new InstantCommand(() -> intakeRoll.stop()));
    
            // driverController.b()
            //         .whileTrue(
            //                 new InstantCommand(() -> intakeRoll.setSpeed(-0.1)))
            //         .onFalse(
            //                 new InstantCommand(() -> intakeRoll.stop()));
    
            // driverController.b()
            //         .onTrue(
            //                 new ZeroPCM(arm));
    
            // driverController.x()
            //         .onTrue(
            //                 new MoveArmToGrabAlgaeCommand(arm));
    
            // driverController.a()
            //         .onTrue(
            //                 new MoveArmToGrabCoralCommand(arm));
    
            // driverController.y()
            //         .onTrue(
            //                 new MoveArmToHumanLoadCommand(arm));
    
            driverController.b()
                    .onTrue(
                            new ParallelCommandGroup(
                                    new ZeroPCM(elevator),
                                    new ZeroPCM(arm),
                                    new ZeroPCM(climber)
                            )
                            );
    
            driverController.povRight()
                    .onTrue(
                            // new RunRollCommand(intakeRoll));
                            new DeployIntakeCommand(intakeFlop).withTimeout(1.5).andThen(new RunRollCommand(intakeRoll)));
                        //     new DeployClimberCommand(climber));[]\
        driverController.povLeft()
                .onTrue(
                        new StopRollCommand(intakeRoll).andThen(new RetractIntakeCommand(intakeFlop)));
                

    
            driverController.a()
                    .onTrue(
                            // new StopRollCommand(intakeRoll).andThen(new RetractIntakeCommand(intakeFlop)));
                            new MoveArmToScoreCommand(arm));
                        // new InstantCommand(() -> {
                        //         intakeRoll.setSpeed(0.5);
                        // })
                //     );
    
            driverController.y()
                    .onTrue(
                            new DeployElevatorL4Command(elevator));
            
            driverController.povUp().whileTrue(
                // new InstantCommand(() -> {
                //         gripper.setCoralSpeed(.5);
                // }));
                //     new GripCoral(gripper));
                new GripCoral(gripper));

                


        // // Target adjustment bindings
        // driverController.povUp().whileTrue(
        //         new MoveArmToNetCommand(arm));
        driverController.povDown().whileTrue(
                new ReleaseCoral(gripper));
        // driverController.povLeft().whileTrue(
        //         new MoveArmToScoreL1Command(arm));
        // driverController.povRight().whileTrue(
        //         new DeployElevatorL4Command(elevator));

    }

    private void configureDrivetrainBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
         drivetrain.setDefaultCommand(
                 // Drivetrain will execute this command periodically
                 drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                    // negative Y
                                                                                                    // (forward)
                         .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                         .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                     // negative X (left)
                 ));

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(
        // () -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(),
        // -driverController.getLeftX()))));
        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        // SmartDashboard.putData("Auto Mode", autoChooser);
        // swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName));

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * driverControllers}.
         */

        // driverController.a().whileTrue(swerve.applyRequest(() -> brake));
        // driverController.b().whileTrue(swerve.applyRequest(
        //         () -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(),
        //                 -driverController.getLeftX()))));

         double ID = 18;
         double distanceOffset = 1.5;
         double coralOffset = REEF_SIDE * -1 / 2;
        AprilTagFieldLayout fieldLayout =
         AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        driverController.x().onTrue(drivetrain.getPathPlannerCommandToAprilTag(new Pose2d(
         fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getX() +
         Math.cos(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*distanceOffset,
         fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getY() +
         Math.sin(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*distanceOffset,
         new
         Rotation2d(fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getRotation().getRadians()
         - Math.PI)
         )));
         driverController.rightBumper().onTrue(drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName))));


        // driverController.y().onTrue(new ResetGyro(swerve, seaweed,
        // pidgey).withTimeout(0.75).andThen(new AlignCommand(swerve, seaweed,
        // pidgey)).withTimeout(5));

         driverController.rightBumper().and(driverController.povUp()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0))));
         driverController.rightBumper().and(driverController.povUpRight()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(-.5))));
         driverController.rightBumper().and(driverController.povUpLeft()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(.5))));

         driverController.rightBumper().and(driverController.povDown()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0))));
         driverController.rightBumper().and(driverController.povDownRight()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(-.5))));
         driverController.rightBumper().and(driverController.povDownLeft()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(.5))));

         driverController.rightBumper().and(driverController.povRight()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5))));
         driverController.rightBumper().and(driverController.povLeft()
                 .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0))));

        // // reset the field-centric heading on left bumper press
         driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

         drivetrain.registerTelemetry(logger::telemeterize);
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

        // If the motor is currently moving, update the target immediately
        // if (driverController.rightBumper().getAsBoolean()) {
        // climber.positionControlledMotor.goToPosition(this.targetPosition);
        // }
    }
}
