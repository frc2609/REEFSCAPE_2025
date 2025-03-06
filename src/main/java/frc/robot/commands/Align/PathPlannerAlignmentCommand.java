package frc.robot.commands.Align;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathPlannerAlignmentCommand extends Command {
    private final CommandSwerveDrivetrain m_Swerve;
    private String limeLightName;
    // private AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public PathPlannerAlignmentCommand(CommandSwerveDrivetrain swerve) {
        m_Swerve = swerve;
        // fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        
        //double ID = LimelightHelpers.getFiducialID(limeLightName);
        double ID = 21;
        double offset = 1;
        LimelightHelpers.setFiducial3DOffset(limeLightName, 1, 1, 0);

        m_Swerve.getPathPlannerCommandToAprilTag(new Pose2d(
        // fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getX() + Math.cos(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*offset,
        // fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d().getY() + Math.sin(fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().getAngle())*offset,
        // fieldLayout.getTagPose((int)Math.round(ID)).get().getRotation().toRotation2d()
        ));


        // System.out.println(fieldLayout.getTagPose((int)Math.round(ID)).get().toPose2d());
    }
}