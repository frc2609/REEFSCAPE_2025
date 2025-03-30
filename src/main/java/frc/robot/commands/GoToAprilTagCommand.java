package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoToAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final int m_tagId;
    private AprilTagFieldLayout m_fieldLayout;
    private Command m_pathCommand;
    
    public GoToAprilTagCommand(CommandSwerveDrivetrain drivetrain, int tagId) {
        m_drivetrain = drivetrain;
        m_tagId = tagId;
        m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        // Get current values from elastic/SmartDashboard
        double xOffset = SmartDashboard.getNumber("TagNav/Tag" + m_tagId + "/XOffset", 0.5);
        double yOffset = SmartDashboard.getNumber("TagNav/Tag" + m_tagId + "/YOffset", 0.5);
        double rotOffset = SmartDashboard.getNumber("TagNav/Tag" + m_tagId + "/RotationOffset", 180);
        
        // Create the path to the tag
        Pose2d tagPose = m_fieldLayout.getTagPose(m_tagId).get().toPose2d();
        
        Transform2d offsetTransform = new Transform2d(
            xOffset, 
            yOffset,
            Rotation2d.fromDegrees(rotOffset)
        );
        
        Pose2d targetPose = tagPose.transformBy(offsetTransform);
        
        // Start the path command
        m_pathCommand = m_drivetrain.getPathPlannerCommandToAprilTag(targetPose);
        m_pathCommand.initialize();
    }
    
    @Override
    public void execute() {
        if (m_pathCommand != null) {
            m_pathCommand.execute();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (m_pathCommand != null) {
            m_pathCommand.end(interrupted);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Command runs until the button is released or the path is completed
        return m_pathCommand != null && m_pathCommand.isFinished();
    }
} 