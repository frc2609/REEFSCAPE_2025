package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class PathToAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private Command pathCommand;

    public PathToAprilTagCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        // Get the current detected AprilTag pose
        var validTarget = limelight.get_tv();

        if (validTarget) {
            // Get the first detected tag's pose in field coordinates
            Pose2d targetPose = limelight.get_Pose2d();

            // Generate path to the target pose
            pathCommand = drivetrain.getPathPlannerCommandToPose(targetPose);

            // Schedule the path command
            pathCommand.schedule();
        } else {
            // If no tags found, end the command
            System.out.println("No AprilTags detected!");
            cancel();
        }
    }

    @Override
    public void execute() {
        // Path following is handled by the scheduled path command

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Path to AprilTag was interrupted!");
        }
        if (pathCommand != null) {
            pathCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand != null && pathCommand.isFinished();
    }
}