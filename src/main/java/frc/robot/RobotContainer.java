// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.PathToAprilTagCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.MoveElevatorToPositionSDB;
import frc.robot.commands.climber.MoveClimberToPosition;
import frc.robot.commands.climber.MoveClimberToPositionSDB;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Telemetry;

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
   private final Elevator elevator = new Elevator();
    // private final Climber climber = new Climber();
    // private final Intake intake = new Intake(); 

    private static double targetPosition = 0.1;

    public RobotContainer() {

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
        // pidgey.clearStickyFault_BootDuringEnable();

        configureBindings();


    }

    private void configureBindings() {
     
        // joystick.a()
        // .whileTrue(
        //     new InstantCommand(() -> intake.setSpeed(0.1))

        //     ).onFalse(
        //         new InstantCommand(() -> intake.stop())
        //     );
    
        // joystick.b()
        // .whileTrue(
        //     new InstantCommand(() -> intake.setSpeed(-0.1))
        
        //     ).onFalse(
        //         new InstantCommand(() -> intake.stop())
        //     ); 


        

        joystick.rightBumper()
        .whileTrue(
            new MoveElevatorToPositionSDB(elevator)
        );

        joystick.povUp()
        .onTrue(
            new InstantCommand(() -> {
                targetPosition += 0.01;
                SmartDashboard.putNumber("Target", targetPosition);
            })
        );

        joystick.povDown()
        .onTrue(
            new InstantCommand(() -> {
                targetPosition -= 0.01;
                SmartDashboard.putNumber("Target", targetPosition);
            })
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
