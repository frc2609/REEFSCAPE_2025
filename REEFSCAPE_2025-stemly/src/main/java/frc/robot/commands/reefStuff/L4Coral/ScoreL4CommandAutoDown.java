package frc.robot.commands.reefStuff.L4Coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class ScoreL4CommandAutoDown extends SequentialCommandGroup{
    public ScoreL4CommandAutoDown(Elevator elevator, Arm arm, Gripper gripper){
        addCommands(
            new ParallelCommandGroup(
         new MovePCM(elevator, 8.5),
         new MovePCM(arm, 0)
            )
        );
    }
}