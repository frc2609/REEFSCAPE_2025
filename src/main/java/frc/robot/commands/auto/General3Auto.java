package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Align.PIDFineAlign;
import frc.robot.commands.Intake.HumanIntakeCommandAuto;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4CommandAuto;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4CommandAutoDown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.utils.LimelightHelpers;

public class General3Auto extends SequentialCommandGroup{
    SwerveRequest.RobotCentric rocDrive = new SwerveRequest.RobotCentric().withVelocityX(-1);
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    CommandSwerveDrivetrain drivetrain;
    private final String limelightName = "limelight-intake";
    public General3Auto(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Gripper gripper, double distanceOffset, int coralStation, int secondReef, int thirdReef, boolean leftSecond, boolean leftThird) {
        this.drivetrain = drivetrain;
        addCommands(
            // Start aligning to visible tag
            new PIDFineAlign(true, drivetrain).withTimeout(2.5),

            // Try adding a conditional command aound the rest that uses a trigger checking for a tag
            
            // Attempt to score on the visible tag
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(2.3),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.6),

            // Drive back to reset pos
            drivetrain.applyRequest(() -> rocDrive).withTimeout(0.5),
            new WaitCommand(0.1),
            drivetrain.runOnce(() ->
            drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-intake"))
        ),

            // Path to coralStation and try to get coral
            getPathCommand(2, 4*(Math.PI/3), -0.2, -0.2),
            new HumanIntakeCommandAuto(arm, gripper, elevator).withTimeout(1),

            // Try to add a wait until based on a trigger reading the gripper voltage to detect when a coral is in
            // Path to the 8 april tag
            getPathCommand(8, 7*(Math.PI/6), 2, 1),

            // Align and score to the visible tag
            new PIDFineAlign(false, drivetrain),
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(2.8),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.6),

            // Drive back to reset pose
            drivetrain.applyRequest(() -> rocDrive).withTimeout(0.25),
            new WaitCommand(0.1),
            resetPoseWithLimelight(limelightName),

            // Path to coralStation and try to get coral
            getPathCommand(2, Math.PI/3, 0.1, -0.2),
            new HumanIntakeCommandAuto(arm, gripper, elevator).withTimeout(1),

            // Try to add a wait until based on a trigger reading the gripper voltage to detect when a coral is in
            // Path to the 8 april tag
            getPathCommand(8,-Math.PI/3, 2, 1),

            // Align and score to the visible tag
            new PIDFineAlign(true, drivetrain),
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(2.8),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.6)

        );
    }

    private Command resetPoseWithLimelight(String limelightName) {
        return drivetrain.runOnce(() ->
            drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiRed(limelightName))
        );
    }

    private Command getPathCommand(int apriltag, double rotationOffset, double distanceOffset, double extraOffset) {
        Pose2d tagPose = fieldLayout.getTagPose(apriltag).get().toPose2d();
        double xOffset = Math.cos(tagPose.getRotation().getRadians())*distanceOffset;
        double yOffset = Math.sin(tagPose.getRotation().getRadians())*distanceOffset;
        double extraXOffset = Math.sin(tagPose.getRotation().getRadians())*extraOffset;
        double extraYOffset = Math.cos(tagPose.getRotation().getRadians())*extraOffset;
        // Transform2d offsetTransform = new Transform2d(
        //     xOffset, 
        //     yOffset,
        //     Rotation2d.fromDegrees(rotationOffset)
        // );
        Pose2d targetPose;
        // Pose2d targetPose = tagPose.transformBy(offsetTransform);
        if(apriltag != 2 || apriltag != 1 || apriltag != 13 || apriltag != 12){
            targetPose = new Pose2d(
                tagPose.getX() + xOffset,
                tagPose.getY() + yOffset,
                new Rotation2d(rotationOffset)
                
            );
        }else{
            targetPose = new Pose2d(
                tagPose.getX() + xOffset + extraXOffset,
                tagPose.getY() + yOffset + extraYOffset,
                new Rotation2d(rotationOffset)
                
            );
        }
        
        return drivetrain.getPathPlannerCommandToAprilTag(targetPose);
    }

// There seems to be an error in the math on this one. You should be doing the multiplication on the outside of the cos and sin
/*
    private Command getPathCommand(int apriltag, double translationOffset, double rotationOffset) {
        return drivetrain.getPathPlannerCommandToAprilTag(
            new Pose2d(
                fieldLayout.getTagPose(apriltag).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose(apriltag).get().getRotation().getAngle()*translationOffset),
                fieldLayout.getTagPose(apriltag).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose(apriltag).get().getRotation().getAngle()*translationOffset),
                new Rotation2d(
                    fieldLayout.getTagPose(apriltag).get().toPose2d().getRotation().getRadians() + rotationOffset
                )
            )
        );
    }
*/
}
