package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Align.ReefPIDAlign;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4CommandAuto;
import frc.robot.commands.reefStuff.L4Coral.ScoreL4CommandAutoDown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.utils.LimelightHelpers;

public class PracticeHumanAuto extends SequentialCommandGroup{
    public PracticeHumanAuto(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Gripper gripper, double distanceOffset, int coralStation, int firstReef, int secondReef){
        addCommands(
            new ReefPIDAlign(drivetrain).withTimeout(3.2),
            drivetrain.runOnce(() ->drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"))),
            new ScoreL4CommandAuto(elevator, arm, gripper).withTimeout(4),
            new ScoreL4CommandAutoDown(elevator, arm, gripper).withTimeout(0.7)

        );
    }
}
