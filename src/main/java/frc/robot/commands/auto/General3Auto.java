package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    public General3Auto(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Gripper gripper, double distanceOffset, int coralStation, int secondReef, int thirdReef){
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        addCommands(
            //drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))),
            new ReefPIDAlign(drivetrain),

            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(2.8),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.6),
            drivetrain.applyRequest(() -> rocDrive).withTimeout(1),
            //Commands.runOnce(()->SmartDashboard.putString("state", "elevator and arm down")),
            new WaitCommand(0.1),
            drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-intake"))),
            //drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))),
            //new WaitCommand(1),
            drivetrain.getPathPlannerCommandToAprilTag(new Pose2d(
                fieldLayout.getTagPose(coralStation).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose(coralStation).get().getRotation().getAngle())*0.50,
                fieldLayout.getTagPose(coralStation).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose(coralStation).get().getRotation().getAngle())*0.50,
                new Rotation2d(fieldLayout.getTagPose(coralStation).get().toPose2d().getRotation().getRadians() - Math.PI/6/* - (Math.PI*2) - (Math.PI/6)*2 + 2*Math.PI/2*/)
                )),
            //drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-intake"))),
            new HumanIntakeCommandAuto(arm, gripper, elevator).withTimeout(2),
            drivetrain.getPathPlannerCommandToAprilTag(new Pose2d(
                fieldLayout.getTagPose(8).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose(8).get().getRotation().getAngle())*1,
                fieldLayout.getTagPose(8).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose(8).get().getRotation().getAngle())*1,
                new Rotation2d(fieldLayout.getTagPose(8).get().toPose2d().getRotation().getRadians() + Math.PI/* - (Math.PI*2) - (Math.PI/6)*2 + 2*Math.PI/2*/)
                )),
                new ReefPIDAlignLeft(drivetrain),

                new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(2.8),
                new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.6)
                    );
    }
}
