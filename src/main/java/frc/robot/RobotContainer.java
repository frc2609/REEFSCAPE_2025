// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.net.PortForwarder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignAndScore;
import frc.robot.commands.CoralHandOff;
import frc.robot.commands.HumanIntakeCommand;
import frc.robot.commands.HumanIntakeCommandAuto;
import frc.robot.commands.PickAlgaeL2Command;
import frc.robot.commands.PickAlgaeL3Command;
import frc.robot.commands.RetractAndHandOff;
import frc.robot.commands.Align.ResetGyro;
import frc.robot.commands.Align.RightAlign;
import frc.robot.commands.Align.LeftAlign;
import frc.robot.commands.ScoreL2Command;
import frc.robot.commands.ScoreL3Command;
import frc.robot.commands.ScoreL4Command;
import frc.robot.commands.ScoreL4CommandAuto;
import frc.robot.commands.ScoreL4CommandAutoDown;
import frc.robot.commands.Intake.DeployIntakeCommand;
import frc.robot.commands.Intake.RetractIntakeCommand;
import frc.robot.commands.Intake.roll.SlowIntakeRollCommand;
import frc.robot.commands.auto.General3Auto;
import frc.robot.commands.auto.ID18_19Station13Blue;
import frc.robot.commands.auto.PracticeCoralAuto;
import frc.robot.commands.auto.PracticeHumanAuto;
import frc.robot.commands.climber.DeployClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.elevator.moveElevator;
import frc.robot.commands.gripper.AutoReleaseGripperCommand;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.gripper.SlowGripCommand;
import frc.robot.commands.gripper.StopGripperCommand;
import frc.robot.commands.pcmUtils.JogPCM;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.commands.Align.LeftAlign;
import frc.robot.commands.Align.PIDAlign;
import frc.robot.commands.Align.PIDFineAlign;
import frc.robot.commands.Align.ReefPIDAlign;
import frc.robot.commands.Align.ReefPIDAlignLeft;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeRoll;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.elastic.Camera;
import frc.robot.subsystems.elastic.FieldDisplay;
import frc.robot.subsystems.elastic.MatchTimeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Telemetry;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;


public class RobotContainer {

    private double currentSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);   
    private double quarterSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/4; 
    private double TopSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);               // kSpeedAt12Volts desired top speed
    private double CurrentAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    private double HalfAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond)/4; // 3/4 of a rotation per second max angular velocity
                                                                                    // max angular velocity
    private final DigitalInput intakeBeam = new DigitalInput(4);

    private SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(currentSpeed * 0.1).withRotationalDeadband(CurrentAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // The robot's subsystems and commands are defined here...

    private final Telemetry logger = new Telemetry(currentSpeed);


    private static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);
  

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private Command queuedCommand = new InstantCommand();

    // /* Path follower */
    public final Arm arm = new Arm();
    public final Elevator elevator = new Elevator();
    public final Climber climber = new Climber();
    public final IntakeRoll intakeRoll = new IntakeRoll();
    public final IntakeFlop intakeFlop = new IntakeFlop();
    public static InstantCommand instantCommand = new InstantCommand();
    private boolean climbed = false;

    public final MatchTimeSubsystem matchTime = new MatchTimeSubsystem();

    private double targetPosition = 0.0;
    private final Pigeon2 pidgey = new Pigeon2(0, "CANivore"); // Pigeon is on roboRIO CAN Bus with device ID 0
    private final String limeLightName = "limelight-intake";
    private Field2d m_field = new Field2d();

    private static double REEF_SIDE = 0.813;

    public final Limelight seaweed = new Limelight("limelight-april");
    private final Trigger elevatorAboveIntake = new Trigger(() -> elevator.getPosition() > 250);
    public final Gripper gripper = new Gripper();
    public SendableChooser<Integer> ID = new SendableChooser<>();
    public boolean isRight = true;

    private final Trigger confirmTrigger= driverController.rightTrigger();// 

    private final Trigger halfSpeedTrigger = driverController.rightBumper();// half speed
    private final Trigger shootTrigger = driverController.povDown();// Gripper outtake
    private final Trigger intakeGroundTrigger = driverController.leftBumper();//Gound intake 
    private final Trigger algaeknockL2Trigger = operatorController.povDown();//L2 algae
    private final Trigger algaeknockL3Trigger = operatorController.povUp();//L3 algae
    private final Trigger resetGyroTrigger = operatorController.start();//rESET GYRO
    private final Trigger alignRightTrigger = driverController.povRight();//Fine align right
    private final Trigger alignLeftTrigger = driverController.povLeft();//Fine align left
    private final Trigger coralHandoffTrigger = operatorController.x();//Coral handoff
    private final Trigger scoreL4Trigger = operatorController.y();//L4 coral Score
    private final Trigger scoreL3Trigger = operatorController.b();//l3 coral score
    private final Trigger scoreL2Trigger = operatorController.a();//L2 coral score
    private final Trigger humanTrigger = driverController.leftTrigger();//Human intake
    private final Trigger gripTrigger = operatorController.leftBumper();//Gripper outake (manual)
    private final Trigger resetYawTrigger = driverController.start();//Reset yaw
    private final Trigger deployClimberTrigger = operatorController.leftTrigger();//Deploy climber
    private final Trigger retractClimberTrigger = operatorController.rightTrigger();//Retract climber
    private final Trigger interupTrigger = driverController.a();
    private final Trigger climbing = new Trigger(() -> climber.getPosition() > 100);
    double distanceOffset = 0.25;
    double coralOffset = 0.27;
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public final FieldDisplay fieldDisplay = new FieldDisplay();

    SendableChooser<Integer> IDCoral = new SendableChooser<>();
    SendableChooser<Integer> IDReef1 = new SendableChooser<>();
    SendableChooser<Integer> IDReef2 = new SendableChooser<>();
    /**
     * The container for the robot.f Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {  
        SmartDashboard.putString("Queue:", "None");
        elevator.setDefaultCommand(new MovePCM(elevator, 8.5));
        arm.setDefaultCommand(new MovePCM(arm, 0));
        gripper.setDefaultCommand(new SlowGripCommand(gripper));
        intakeRoll.setDefaultCommand(new SlowIntakeRollCommand(intakeRoll));
       
        SmartDashboard.putNumber("Distance Offset", 0.25);

        new EventTrigger("PID Align").onTrue(new SequentialCommandGroup(new ReefPIDAlign(drivetrain).withTimeout(3),drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))), new ScoreL4CommandAuto(elevator, arm, gripper).andThen(new WaitCommand(1)).andThen(new ScoreL4CommandAutoDown(elevator, arm, gripper))));
        NamedCommands.registerCommand("PID Align", new SequentialCommandGroup(new ReefPIDAlign(drivetrain).withTimeout(3),drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))), new ScoreL4CommandAuto(elevator, arm, gripper).andThen(new WaitCommand(1)).andThen(new ScoreL4CommandAutoDown(elevator, arm, gripper))));

 
        distanceOffset = SmartDashboard.getNumber("Distance Offset", 0.25);

        boolean jog = false;
        IDCoral.setDefaultOption("1", 1);
        IDCoral.addOption("1", 1);
        IDCoral.addOption("2", 2);
        IDCoral.addOption("3", 3);
        IDCoral.addOption("4", 4);
        IDCoral.addOption("5", 5);
        IDCoral.addOption("6", 6);
        IDCoral.addOption("7", 7);
        IDCoral.addOption("8", 8);
        IDCoral.addOption("9", 9);
        IDCoral.addOption("10", 10);
        IDCoral.addOption("11", 11);
        IDCoral.addOption("12", 12);
        IDCoral.addOption("13", 13);
        IDCoral.addOption("14", 14);
        IDCoral.addOption("15", 15);
        IDCoral.addOption("16", 16);
        IDCoral.addOption("17", 17);
        IDCoral.addOption("18", 18);
        IDCoral.addOption("19", 19);
        IDCoral.addOption("20", 20);
        IDCoral.addOption("21", 21);
        IDCoral.addOption("22", 22);

        IDReef1.setDefaultOption("1", 1);
        IDReef1.addOption("1", 1);
        IDReef1.addOption("2", 2);
        IDReef1.addOption("3", 3);
        IDReef1.addOption("4", 4);
        IDReef1.addOption("6", 6);
        IDReef1.addOption("5", 5);
        IDReef1.addOption("7", 7);
        IDReef1.addOption("8", 8);
        IDReef1.addOption("9", 9);
        IDReef1.addOption("10", 10);
        IDReef1.addOption("11", 11);
        IDReef1.addOption("12", 12);
        IDReef1.addOption("13", 13);
        IDReef1.addOption("14", 14);
        IDReef1.addOption("15", 15);
        IDReef1.addOption("16", 16);
        IDReef1.addOption("17", 17);
        IDReef1.addOption("18", 18);
        IDReef1.addOption("19", 19);
        IDReef1.addOption("20", 20);
        IDReef1.addOption("21", 21);
        IDReef1.addOption("22", 22);

        IDReef2.setDefaultOption("1", 1);
        IDReef2.addOption("1", 1);
        IDReef2.addOption("2", 2);
        IDReef2.addOption("3", 3);
        IDReef2.addOption("4", 4);
        IDReef2.addOption("6", 6);
        IDReef2.addOption("5", 5);
        IDReef2.addOption("7", 7);
        IDReef2.addOption("8", 8);
        IDReef2.addOption("9", 9);
        IDReef2.addOption("10", 10);
        IDReef2.addOption("11", 11);
        IDReef2.addOption("12", 12);
        IDReef2.addOption("13", 13);
        IDReef2.addOption("14", 14);
        IDReef2.addOption("15", 15);
        IDReef2.addOption("16", 16);
        IDReef2.addOption("17", 17);
        IDReef2.addOption("18", 18);
        IDReef2.addOption("19", 19);
        IDReef2.addOption("20", 20);
        IDReef2.addOption("21", 21);
        IDReef2.addOption("22", 22);
        SmartDashboard.putData("ID Coral Station", IDCoral);
        SmartDashboard.putData("ID First Reef", IDReef1);
        SmartDashboard.putData("ID Second Reef", IDReef2);
        
        if (jog == true){
            configureJogBindings();
        } else {
            configureBindings();    
        }

        configureDrivetrainBindings();
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);

        drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName));

    }

    private void configureJogBindings() {

        operatorController.a().and(operatorController.povUp()).onTrue(new JogPCM(arm, 15));
        operatorController.a().and(operatorController.povDown()).onTrue(new JogPCM(arm, -15));

        operatorController.x().and(operatorController.povUp()).onTrue(new JogPCM(climber, 1));
        operatorController.x().and(operatorController.povDown()).onTrue(new JogPCM(climber, -1));

        operatorController.y().and(operatorController.povUp()).onTrue(new JogPCM(elevator, 1));
        operatorController.y().and(operatorController.povDown()).onTrue(new JogPCM(elevator, -1));

        operatorController.b().and(operatorController.povUp()).onTrue(new JogPCM(intakeFlop, 1));
        operatorController.b().and(operatorController.povDown()).onTrue(new JogPCM(intakeFlop, -1));
    }

    private void configureBindings() {

        // intakeFlop.setDefaultCommand(new RetractIntakeCommand(intakeFlop, intakeRoll));
        elevator.setDefaultCommand(new MovePCM(elevator, 8.5));
        arm.setDefaultCommand(new MovePCM(arm, 0));
        gripper.setDefaultCommand(new SlowGripCommand(gripper));

        interupTrigger.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        
        climbing.toggleOnTrue(
            new moveElevator(elevator)
        );
      
        halfSpeedTrigger
            .whileTrue(Commands.runOnce(()-> currentSpeed = quarterSpeed))
            .whileFalse(Commands.runOnce(()-> currentSpeed =TopSpeed))
            .whileTrue(Commands.runOnce(()-> CurrentAngularRate = HalfAngularRate))
            .whileFalse(Commands.runOnce(()-> CurrentAngularRate = MaxAngularRate));
            
        intakeGroundTrigger
            .whileTrue(new DeployIntakeCommand(intakeFlop, intakeRoll))
            .whileFalse(new RetractIntakeCommand(intakeFlop, intakeRoll));
        intakeFlop.coralTrigger.onTrue(
            new RetractIntakeCommand(intakeFlop, intakeRoll)
        );
        humanTrigger.whileTrue(new HumanIntakeCommand(arm, gripper, elevator));
        alignLeftTrigger.whileTrue(new LeftAlign(drivetrain));
        alignRightTrigger.whileTrue(new RightAlign(drivetrain));
        // alignRightTrigger.onFalse(
        //     queuedCommand = new InstantCommand()
        // );
        coralHandoffTrigger.onTrue(new CoralHandOff(elevator, arm, gripper));
        resetYawTrigger.onTrue(new ResetGyro(drivetrain, seaweed, pidgey));
        resetGyroTrigger.onTrue(new ResetGyro(drivetrain, seaweed, pidgey));
        
        boolean l4queue = false;
        boolean l3queue = false;
        boolean l2queue = false;

        scoreL4Trigger.toggleOnTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L4 Coral")),
                new ScoreL4Command(elevator, arm, gripper, shootTrigger, confirmTrigger)
            )
        );

        if(scoreL4Trigger.getAsBoolean()){
            scoreL4Trigger.onTrue(Commands.runOnce(() -> SmartDashboard.putString("Queue:", "unqued")));
        }else{

        }

       confirmTrigger.onTrue(Commands.runOnce(() -> SmartDashboard.putString("Queue:", "None")));
        //scoreL4Trigger.onTrue(new AlignAndScore(new ScoreL4CommandAuto(elevator, arm, gripper), drivetrain, alignRightTrigger, alignLeftTrigger));

        scoreL3Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L3 Coral")),
                new ScoreL3Command(elevator, arm, gripper, shootTrigger, confirmTrigger)
            )
        );
        scoreL2Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L2 Coral")),
                new ScoreL2Command(elevator, arm, gripper, shootTrigger, confirmTrigger)
            )
        );
        algaeknockL2Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L2 Algae")),
                new PickAlgaeL2Command(elevator, arm, gripper, confirmTrigger)
            )
        );
        algaeknockL3Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L3 Algae")),
                new PickAlgaeL3Command(elevator, arm, gripper, confirmTrigger)
            )
        );
        deployClimberTrigger
            .whileTrue(new DeployClimberCommand(climber))
            .whileFalse(new RetractClimberCommand(climber)); 
        gripTrigger.whileTrue(new ReleaseGripperCommand(gripper));
        
    }
    
    public double smootherJoystick(double x) {
        x = Math.max(-1.0, Math.min(1.0, x));
        double absX = Math.abs(x);
        // SmootherStep: 6|x|^5 - 15|x|^4 + 10|x|^3
        double smooth = 6 * Math.pow(absX, 5) - 15 * Math.pow(absX, 4) + 10 * Math.pow(absX, 3);
        return Math.copySign(smooth, x);
    }
    private void configureDrivetrainBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX((-driverController.getLeftY()) * currentSpeed) 
                .withVelocityY((-driverController.getLeftX()) * currentSpeed) 
                .withRotationalRate(-driverController.getRightX() * CurrentAngularRate)
        ));

        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        operatorController.back().onTrue(drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limeLightName))));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private double smooth(double value){
        double absValue = Math.abs(value);

        double a = 6 * Math.pow(absValue, 8.5);
        double b = 15 * Math.pow(absValue, 6.8);
        double c = 10 * Math.pow(absValue, 5.1);

        return Math.copySign(a - b + c, value);
    }

    public void robotInit() {
        for (int port = 5800; port <= 5810; port++) {
            PortForwarder.add(port, "limelight-april.local", port);
        }

    }

    public Command getAutonomousCommand() {
        return new General3Auto(drivetrain, elevator, arm, gripper, SmartDashboard.getNumber("Distance Offset", 0.25), IDCoral.getSelected(), IDReef1.getSelected(), IDReef2.getSelected());
        //return autoChooser.getSelected();
        //return new PracticeCoralAuto(drivetrain, elevator, arm, gripper, distanceOffset, 0, 0, 0);
        //return new PracticeHumanAuto(drivetrain, elevator, arm, gripper, distanceOffset, 0, 0, 0);
    }
    
}