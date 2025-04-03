// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
 * To Do:
 * 1. Remove unneeded commands
 * 2. If we need the ResetGyro command, lets not use a limelight that does not exist also... 
 *    - WHY IS IT DOING PID?!
 *    - WHY ARE THERE TWO OF THESE CLASSES?!
 * 3. Rebind or remove line 251
 * 4. Do a ctrl-shift-f for 'limelight-april' 
 *    - WHO IS APRIL AND WHY ARE WE USING THEIR LIMELIGHT?!
 * 5. Why is the reset yaw calling reset gyro on line 248?
 * 6. Test General3Auto and read the comments
 */

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.commands.GoToAprilTagCommand;
import frc.robot.commands.Align.PIDFineAlign;
import frc.robot.commands.Align.ResetGyro;

import frc.robot.commands.Intake.roll.SlowIntakeRollCommand;
import frc.robot.commands.Intake.roll.IntakeRollCommand;
import frc.robot.commands.Intake.flop.DeployFlopCommand;
import frc.robot.commands.Intake.RetractIntakeCommand;
import frc.robot.commands.Intake.HumanIntakeCommand;
import frc.robot.commands.Intake.CoralHandOff;
import frc.robot.commands.auto.DchampsBlueLeft;
import frc.robot.commands.auto.DchampsBlueRight;
import frc.robot.commands.auto.DchampsRedLeft;
import frc.robot.commands.auto.DchampsRedRight;
//import frc.robot.commands.auto.MultiTagPathAuto;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.StopClimberCommand;
import frc.robot.commands.climber.DeployClimberCommand;

import frc.robot.commands.elevator.moveElevator;

import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.gripper.SlowGripCommand;

import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.commands.pcmUtils.JogPCM;

import frc.robot.commands.reefStuff.Algae.PickAlgaeL3Command;
import frc.robot.commands.reefStuff.Algae.PickAlgaeL2Command;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4Command;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4KnockAlgaeL2Command;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4KnockAlgaeL3Command;
import frc.robot.commands.reefStuff.Algae.AlgaeL2LED;
import frc.robot.commands.reefStuff.Algae.AlgaeL3LED;
import frc.robot.commands.reefStuff.ScoreL2Command;
import frc.robot.commands.reefStuff.ScoreL3Command;
import frc.robot.commands.reefStuff.L4Coral.L4LED;
import frc.robot.commands.reefStuff.L4Coral.LED_track;
import frc.robot.commands.reefStuff.NoCoralLED;
import frc.robot.commands.reefStuff.L2LED;
import frc.robot.commands.reefStuff.L3LED;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRoll;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Led;

import frc.robot.utils.SendableChooserUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Telemetry;



public class RobotContainer {
    /* Controllers */
    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);


    /* Drive */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private final int speedFactor = 4;

    private double topDriveSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);  
    private double slowDriveSpeed = topDriveSpeed/speedFactor; 
    private double currentSpeed = topDriveSpeed;   
    
    private double topAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    private double slowAngularRate = topAngularRate/speedFactor;
    private double currentAngularRate = topAngularRate;
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(currentSpeed);
    

    /* Subsystems */
    public final Arm arm = new Arm();
    public final Gripper gripper = new Gripper();
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    public final IntakeRoll intakeRoll = new IntakeRoll();
    public final IntakeFlop intakeFlop = new IntakeFlop();
    public final Led led = new Led(intakeFlop.coralTrigger);
    public final Pigeon2 pidgey = new Pigeon2(0, "CANivore");

    private final String fineLimelightName = "limelight";
    private final String mainLimelightName = "limelight-intake";
    public final Limelight mainLimelight = new Limelight(mainLimelightName);
    public final Limelight fineLimelight = new Limelight(fineLimelightName);
    
    
    /* Triggers */
    @SuppressWarnings("unused")
    private final Trigger scoreL1Trigger = operatorController.x();//L2 coral score
    private final Trigger retractClimberTrigger = operatorController.rightTrigger();//Retract climber
    private final Trigger deployClimberTrigger = operatorController.leftTrigger();//Deploy climber
    private final Trigger coralHandoffTrigger = operatorController.rightBumper();//Coral handoff
    private final Trigger algaeknockL2Trigger = operatorController.povDown();//L2 algae
    private final Trigger algaeknockL3Trigger = operatorController.povUp();//L3 algae
    //private final Trigger interupTrigger = operatorController.back();
    private final Trigger resetGyroTrigger = operatorController.start();//rESET GYRO
    private final Trigger gripTrigger = operatorController.leftBumper();//Gripper outake (manual)
    private final Trigger scoreL4Trigger = operatorController.y();//L4 coral Score
    private final Trigger scoreL4TriggerL3Algae = operatorController.povLeft();//L4 coral Score
    private final Trigger scoreL4TriggerL2Algae = operatorController.povRight();//L4 coral Score
    private final Trigger scoreL3Trigger = operatorController.b();//l3 coral score
    private final Trigger scoreL2Trigger = operatorController.a();//L2 coral score
    
    private final Trigger intakeGroundTrigger = driverController.leftBumper();//Gound intake 
    private final Trigger halfSpeedTrigger = driverController.rightBumper();// half speed
    private final Trigger confirmTrigger= driverController.rightTrigger();// 
    private final Trigger alignRightTrigger = driverController.povRight();//Fine align right
    private final Trigger alignLeftTrigger = driverController.povLeft();//Fine align left
    private final Trigger humanTrigger = driverController.leftTrigger();//Human intake
    private final Trigger resetYawTrigger = driverController.start();//Reset yaw
    private final Trigger shootTrigger = driverController.povDown();// Gripper outtake
    
    /* Auto choosers */
    SendableChooser<Integer> IDCoral = SendableChooserUtil.createSequentialChooser();
    SendableChooser<Integer> IDReef2 = SendableChooserUtil.createSequentialChooser();
    SendableChooser<Integer> IDReef3 = SendableChooserUtil.createSequentialChooser();

    SendableChooser<Command> autoChooser = new SendableChooser<>(); 

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);




    /**
     * The container for the robot.f Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() { 
        setSmartDashboard();

        boolean jog = false;
        boolean sysid = false;

        if (sysid){
            configureSysidBindings();
            
        } else if (jog) {
            configureJogBindings();
            
        } else {
            configureDefaultCommands();
            configureBindings();    
        }

        configureDrivetrainBindings();
    }
    
    /**
     * You will have to go into CommandSwerveDrivetrain and 
     * change m_sysIdRoutineToApply to the three different methods
     * 1. m_sysIdRoutineSteer
     * 2. m_sysIdRoutineTranslation
     * 3. m_sysIdRoutineRotation
     *  
     * Run the following bindings in order!
     * Each should be run exactly once in a single log!
     */
    private void configureSysidBindings() {        
        // first
        operatorController.start().and(operatorController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // second
        operatorController.start().and(operatorController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // third
        operatorController.back().and(operatorController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // fourth
        operatorController.back().and(operatorController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        
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
    
    private void configureDefaultCommands() {
        elevator.setDefaultCommand(new MovePCM(elevator, 8.5));
        gripper.setDefaultCommand(new SlowGripCommand(gripper));
        arm.setDefaultCommand(new MovePCM(arm, 0));
        led.setDefaultCommand(new NoCoralLED(led));
        intakeRoll.setDefaultCommand(new SlowIntakeRollCommand(intakeRoll));
    }
    
    private void configureBindings() {
        //interupTrigger.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        
        confirmTrigger.onTrue(Commands.runOnce(() -> SmartDashboard.putString("Queue:", "None")));
        
        humanTrigger.whileTrue(new HumanIntakeCommand(arm, gripper, elevator));
        
        alignLeftTrigger.whileTrue(new PIDFineAlign(true, drivetrain));
        alignRightTrigger.whileTrue(new PIDFineAlign(false, drivetrain));
        
        coralHandoffTrigger.onTrue(new CoralHandOff(elevator, arm, gripper));
        
        resetYawTrigger.onTrue(new ResetGyro(drivetrain, mainLimelight, pidgey));
        resetGyroTrigger.onTrue(drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-intake"))));
        
        // driverController.x().onTrue(drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiRed("limelight"))));
        driverController.back().onTrue(drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(mainLimelightName))));

        gripTrigger.whileTrue(new ReleaseGripperCommand(gripper));
        
        climber.climbing.whileTrue(new moveElevator(elevator));
        deployClimberTrigger
            .whileTrue(
                new SequentialCommandGroup(
                    climber.runOnce(() -> climber.enablePositionControl()),
                    new DeployClimberCommand(climber)
                )
            )
            .onFalse(new StopClimberCommand(climber)); 

        retractClimberTrigger
            .whileTrue(
                new SequentialCommandGroup(
                    climber.runOnce(() -> climber.enablePositionControl()),
                    new RetractClimberCommand(climber)
                )
            )
            .onFalse(new StopClimberCommand(climber));
      
        halfSpeedTrigger
            .whileTrue(Commands.runOnce(()-> {
                currentSpeed = slowDriveSpeed;
                currentAngularRate = slowAngularRate;
            }))
            .whileFalse(Commands.runOnce(()-> {
                currentSpeed =topDriveSpeed;
                currentAngularRate = topAngularRate;
            }));
            
        intakeGroundTrigger.debounce(0.1)
            .whileTrue(new DeployFlopCommand(intakeFlop))
            .whileTrue(new IntakeRollCommand(intakeRoll))
            .whileFalse(new RetractIntakeCommand(intakeFlop, intakeRoll));

        intakeFlop.coralTrigger
        .and(intakeFlop.deployedTrigger)
        .onTrue(
            new RetractIntakeCommand(intakeFlop, intakeRoll)
        );


        intakeFlop.coralTrigger
        .and(intakeFlop.retractedTrigger)
        .and(gripper.coral.negate())
        .onTrue(
            new CoralHandOff(elevator, arm, gripper)
        );

        scoreL4Trigger.toggleOnTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L4 Coral")),
                new ScoreL4Command(elevator, arm, gripper, shootTrigger, alignRightTrigger, alignLeftTrigger)
            )
        );

        scoreL3Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L3 Coral")),
                new ScoreL3Command(elevator, arm, gripper, shootTrigger, alignRightTrigger, alignLeftTrigger)
            )
        );

        algaeknockL3Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L3 Algae")),
                new PickAlgaeL3Command(elevator, arm, gripper, confirmTrigger)
            )
        );

        scoreL2Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L2 Coral")),
                new ScoreL2Command(elevator, arm, gripper, shootTrigger, alignRightTrigger, alignLeftTrigger)
            )
        );
        
        //driverController.x().onTrue(createPathToTag(9, 1, 0, 180));
        
        algaeknockL2Trigger.toggleOnTrue(
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L2 Algae")),
                new PickAlgaeL2Command(elevator, arm, gripper, confirmTrigger)
            )
        );

        scoreL4TriggerL2Algae.toggleOnTrue(          
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L4 coral, L2 Algae")),
                new ScoreL4KnockAlgaeL2Command(elevator, arm, gripper, shootTrigger, alignRightTrigger, alignLeftTrigger)
            )
        );
        scoreL4TriggerL3Algae.toggleOnTrue(          
            new ParallelCommandGroup(
                Commands.runOnce(() -> SmartDashboard.putString("Queue:", "L4 coral, L3 Algae")),
                new ScoreL4KnockAlgaeL3Command(elevator, arm, gripper, shootTrigger, alignRightTrigger, alignLeftTrigger)
            )
        );
        

        scoreL4Trigger.onTrue(new L4LED(led));
        scoreL3Trigger.onTrue(new L3LED(led));
        scoreL2Trigger.onTrue(new L2LED(led));
        algaeknockL3Trigger.onTrue(new AlgaeL3LED(led));
        algaeknockL2Trigger.onTrue(new AlgaeL2LED(led));
        alignLeftTrigger.whileTrue(new LED_track(led));
        alignRightTrigger.whileTrue(new LED_track(led));
        
    }

    private void configureDrivetrainBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-(Math.copySign(Math.pow(driverController.getLeftY(),2),driverController.getLeftY())) * currentSpeed)

                .withVelocityY(-(Math.copySign(Math.pow(driverController.getLeftX(),2),driverController.getLeftX())) * currentSpeed)

                .withRotationalRate(-(Math.copySign(Math.pow(driverController.getRightX(),2),driverController.getRightX())) * currentAngularRate)
        ));

        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        operatorController.back().onTrue(drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(mainLimelightName))));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return new DchampsRedRight(drivetrain, elevator, arm, gripper);
        // return new General3Auto(drivetrain, elevator, arm, gripper, 1, IDCoral.getSelected(), IDReef2.getSelected(), IDReef3.getSelected(), SmartDashboard.getBoolean("left second", false), SmartDashboard.getBoolean("left third", false)); 
        return autoChooser.getSelected();   
    }

    private void setSmartDashboard(){
        SmartDashboard.putNumber("TagNav/Tag8/XOffset", 0.5);
        SmartDashboard.putNumber("TagNav/Tag8/YOffset", 0.5);
        SmartDashboard.putNumber("TagNav/Tag8/RotationOffset", 180);
        SmartDashboard.putNumber("TagNav/Tag2/XOffset", 0.5);
        SmartDashboard.putNumber("TagNav/Tag2/YOffset", 0.5);
        SmartDashboard.putNumber("TagNav/Tag2/RotationOffset", 180);
        SmartDashboard.putNumber("TagNav/Tag/XOffset", 0.5);
        SmartDashboard.putNumber("TagNav/Tag/YOffset", 0.5);
        SmartDashboard.putNumber("TagNav/Tag/RotationOffset", 180);
        SmartDashboard.putNumber("TagNav/Tag/id", 10);
        SmartDashboard.putString("Queue:", "None");        
        SmartDashboard.putData("ID Coral Station", IDCoral);
        SmartDashboard.putData("ID Second Reef", IDReef2);
        SmartDashboard.putData("ID Third Reef", IDReef3);
        SmartDashboard.putBoolean("left second", false);
        SmartDashboard.putBoolean("left third", false);
        autoChooser.addOption("Dchamps Red Right", new DchampsRedRight(drivetrain, elevator, arm, gripper));
        autoChooser.addOption("Dchamps Red Left", new DchampsRedLeft(drivetrain, elevator, arm, gripper));
        autoChooser.addOption("Dchamps Blue Left", new DchampsBlueLeft(drivetrain, elevator, arm, gripper));
        autoChooser.setDefaultOption("Dchamps Blue Right", new DchampsBlueRight(drivetrain, elevator, arm, gripper));

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }
    private Command createPathToTag(int tagId, double xOffset, double yOffset, double rotationOffset) {
        Pose2d tagPose = fieldLayout.getTagPose(tagId).get().toPose2d();
        
        Transform2d offsetTransform = new Transform2d(
            xOffset, 
            yOffset,
            Rotation2d.fromDegrees(rotationOffset)
        );
        
        Pose2d targetPose = tagPose.transformBy(offsetTransform);
        
        return drivetrain.getPathPlannerCommandToAprilTag(targetPose);
    }

}