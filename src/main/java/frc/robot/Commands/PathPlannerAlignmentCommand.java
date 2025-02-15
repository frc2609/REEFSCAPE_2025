package frc.robot.Commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathPlannerAlignmentCommand extends Command {
    private final CommandSwerveDrivetrain m_Swerve;
    private String limeLightName;
    private AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public PathPlannerAlignmentCommand(CommandSwerveDrivetrain swerve) {
        m_Swerve = swerve;
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        addRequirements(swerve);
    }


    public void execute(){
        
        double ID = LimelightHelpers.getFiducialID(limeLightName);    
        m_Swerve.getPathPlannerCommandToAprilTag(new Pose2d(
        fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getX() ,
        fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getY(),
        fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().toRotation2d()
        ));
    }
}