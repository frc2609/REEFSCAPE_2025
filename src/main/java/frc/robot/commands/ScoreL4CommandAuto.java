package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.gripper.AutoReleaseGripperCommand;
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
                new WaitUntilCommand(elevator.aboveIntake),
                new MovePCM(arm, -222),
                new WaitCommand(1),
                new AutoReleaseGripperCommand(gripper).withTimeout(0.5),
                new PrintCommand("SCORE!")
            )            
        );
    }
}