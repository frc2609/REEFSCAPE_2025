package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HumanIntakeCommandAuto;
import frc.robot.commands.ScoreL4CommandAuto;
import frc.robot.commands.ScoreL4CommandAutoDown;
import frc.robot.commands.Align.ReefPIDAlign;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.utils.LimelightHelpers;

public class General3Auto extends SequentialCommandGroup{
    public General3Auto(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Gripper gripper, double distanceOffset, int coralStation, int firstReef, int secondReef){
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        addCommands(
            //drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))),
            new ReefPIDAlign(drivetrain).withTimeout(3.2),
            drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))),
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(3.2),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.7),
            Commands.runOnce(()->SmartDashboard.putString("state", "elevator and arm down")),
            new WaitCommand(0.1),
            drivetrain.getPathPlannerCommandToAprilTag(new Pose2d(
                fieldLayout.getTagPose(coralStation).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose(coralStation).get().getRotation().getAngle())*distanceOffset,
                fieldLayout.getTagPose(coralStation).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose(coralStation).get().getRotation().getAngle())*distanceOffset,
                new Rotation2d(fieldLayout.getTagPose(coralStation).get().toPose2d().getRotation().getRadians() - Math.PI*2)
                )),
            new HumanIntakeCommandAuto(arm, gripper, elevator).withTimeout(2),
            drivetrain.getPathPlannerCommandToAprilTag(new Pose2d(
                fieldLayout.getTagPose(firstReef).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose(firstReef).get().getRotation().getAngle())*1,
                fieldLayout.getTagPose(firstReef).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose(firstReef).get().getRotation().getAngle())*1,
                new Rotation2d(fieldLayout.getTagPose((int)Math.round(firstReef)).get().toPose2d().getRotation().getRadians() - Math.PI)
                )),
            new ReefPIDAlign(drivetrain).withTimeout(3.2),
            drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))),
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(3.2),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.7),
            new WaitCommand(0.1),
            drivetrain.getPathPlannerCommandToAprilTag(new Pose2d(
                fieldLayout.getTagPose(coralStation).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose(coralStation).get().getRotation().getAngle())*distanceOffset,
                fieldLayout.getTagPose(coralStation).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose(coralStation).get().getRotation().getAngle())*distanceOffset,
                new Rotation2d(fieldLayout.getTagPose((int)Math.round(coralStation)).get().toPose2d().getRotation().getRadians() - Math.PI*2)
                )),
            new HumanIntakeCommandAuto(arm, gripper, elevator).withTimeout(2),
            drivetrain.getPathPlannerCommandToAprilTag(new Pose2d(
                fieldLayout.getTagPose(secondReef).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose(secondReef).get().getRotation().getAngle())*1,
                fieldLayout.getTagPose(secondReef).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose(secondReef).get().getRotation().getAngle())*1,
                new Rotation2d(fieldLayout.getTagPose((int)Math.round(secondReef)).get().toPose2d().getRotation().getRadians() - Math.PI)
                )),
            new ReefPIDAlign(drivetrain).withTimeout(4),
            drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))),
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(4),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(1)

        );
    }
}
