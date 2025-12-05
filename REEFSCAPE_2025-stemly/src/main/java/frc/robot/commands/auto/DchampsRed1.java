package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Align.PIDFineAlign;
import frc.robot.commands.Intake.HumanIntakeCommand;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4CommandAuto;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4CommandAutoDown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.utils.LimelightHelpers;

public class DchampsRed1 extends SequentialCommandGroup {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName = "limelight-intake";
    private final AprilTagFieldLayout fieldLayout;
    
    public DchampsRed1(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Gripper gripper) {
        this.drivetrain = drivetrain;
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        
        addCommands(
            // Only proceed if a tag is visible

            // If a tag is visible, execute the sequence
            new WaitCommand(3),
            new SequentialCommandGroup(                
                // First score
                //resetPoseWithLimelight(),
                new ParallelCommandGroup(
                    new PIDFineAlign(true, drivetrain),
                    new ScoreL4CommandAuto(elevator, arm, gripper)
                ).withTimeout(5),
                new ScoreL4CommandAutoDown(elevator, arm, gripper)
                
                
                // If no tag is visible, do nothing (or could add a search behavior)
                // new InstantCommand(),
                // this::isTagVisible
            )
            
        );
    }
    
    private boolean isTagVisible() {
        return LimelightHelpers.getTV(limelightName);
    }
    
    private Command resetPoseWithLimelight() {
        return drivetrain.runOnce(() ->
            drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limelightName))
        );
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