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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Telemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Limelight seaweed = new Limelight("limelight-seaweed");
    private final Pigeon2 pidgey = new Pigeon2(0, "CANivore");

    // /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final Intake intake = new Intake(); 

    private double targetPosition = 0.1;

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        pidgey.clearStickyFault_BootDuringEnable();
        configureDrivetrainBindings();
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



        // Target adjustment bindings
        joystick.povUp().whileTrue(
            new RunCommand(() -> 
                adjustTargetPosition(1)          
        ));
        joystick.povDown().whileTrue(
            new RunCommand(() -> 
                adjustTargetPosition(-1)
        ));
        joystick.povLeft().whileTrue(
            new RunCommand(() -> 
                adjustTargetPosition(-10)
        ));
        joystick.povRight().whileTrue(
            new RunCommand(() -> 
                adjustTargetPosition(10)
        ));
    }

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
    }
}
