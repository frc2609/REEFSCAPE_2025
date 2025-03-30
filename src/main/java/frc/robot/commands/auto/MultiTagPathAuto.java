package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class MultiTagPathAuto extends SequentialCommandGroup {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName = "limelight-intake";
    private final AprilTagFieldLayout fieldLayout;
    
    public MultiTagPathAuto(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        
        addCommands(
            // Only proceed if a tag is visible
            new ConditionalCommand(
                // If a tag is visible, execute the sequence
                new SequentialCommandGroup(
                    // Reset the robot's pose based on what the limelight sees
                    resetPoseWithLimelight(limelightName),
                    
                    // Wait a bit for the pose reset to take effect
                    new WaitCommand(0.1),
                    
                    createPathToTag(9, 0.5, 0.5, 180),
                    
                    // Wait for the robot to reach the first tag
                    new WaitCommand(0.5),
                    
                    // Navigate to tag 2 with rotation=0, x=0.8, y=0.5
                    createPathToTag(2, 0.8, 0.5, 0),

                    createPathToTag(8, 0.5, 0.5, 180),
                    
                    // Wait for the robot to reach the first tag
                    new WaitCommand(0.5),
                    
                    // Navigate to tag 2 with rotation=0, x=0.8, y=0.5
                    createPathToTag(2, 0.8, 0.5, 0)
                ),
                
                // If no tag is visible, do nothing (or could add a search behavior)
                new InstantCommand(),
                this::isTagVisible
            )
        );
    }
    
    private boolean isTagVisible() {
        return LimelightHelpers.getTV(limelightName);
    }
    
    private Command resetPoseWithLimelight(String limelightName) {
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