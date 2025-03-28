package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Align.ReefPIDAlign;
import frc.robot.commands.Align.ReefPIDAlignLeft;
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
    public General3Auto(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Gripper gripper, double distanceOffset, int coralStation, int secondReef, int thirdReef){
        this.drivetrain = drivetrain;
        addCommands(
            // Start aligning to visible tag
            new ReefPIDAlign(drivetrain),

            // Try adding a conditional command aound the rest that uses a trigger checking for a tag
            
            // Attempt to score on the visible tag
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(2.8),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.6),

            // Drive back to reset pose
            drivetrain.applyRequest(() -> rocDrive).withTimeout(1),
            new WaitCommand(0.1),
            resetPoseWithLimelight("limelight-intake"),

            // Path to coralStation and try to get coral
            getPathCommand(coralStation, 0.5, 0.5, -30),
            new HumanIntakeCommandAuto(arm, gripper, elevator).withTimeout(2),

            // Try to add a wait until based on a trigger reading the gripper voltage to detect when a coral is in
            // Path to the 8 april tag
            getPathCommand(8, 1, 1, 180),

            // Align and score to the visible tag
            new ReefPIDAlignLeft(drivetrain),
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(2.8),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.6)
        );
    }

    private Command resetPoseWithLimelight(String limelightName) {
        return drivetrain.runOnce(() ->
            drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiRed(limelightName))
        );
    }

    private Command getPathCommand(int apriltag, double xOffset, double yOffset, double rotationOffset) {
        Pose2d tagPose = fieldLayout.getTagPose(apriltag).get().toPose2d();
        
        Transform2d offsetTransform = new Transform2d(
            xOffset, 
            yOffset,
            Rotation2d.fromDegrees(rotationOffset)
        );
        
        Pose2d targetPose = tagPose.transformBy(offsetTransform);
        
        return drivetrain.getPathPlannerCommandToAprilTag(targetPose);
    }
}
