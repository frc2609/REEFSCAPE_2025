package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class ScoreL4CommandAuto extends ParallelCommandGroup{
    public ScoreL4CommandAuto(Elevator elevator, Arm arm, Gripper gripper){
        addCommands(
            new MovePCM(elevator, 37.5),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new MovePCM(arm, -222),
                new ReleaseGripperCommand(gripper).withTimeout(0.5)
            ),
            Commands.print("level4")
            
        );
    }
}