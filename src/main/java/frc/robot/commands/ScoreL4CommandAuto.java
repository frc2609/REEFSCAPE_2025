package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.gripper.AutoReleaseGripperCommand;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.gripper.SlowGripCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class ScoreL4CommandAuto extends SequentialCommandGroup{
    public ScoreL4CommandAuto(Elevator elevator, Arm arm, Gripper gripper){
        addCommands(
         new MovePCM(elevator, 8.5),
         new MovePCM(arm, 0),
         new SlowGripCommand(gripper),
            new ParallelCommandGroup(
            new MovePCM(elevator, 38), //39
            new SequentialCommandGroup(
                new WaitUntilCommand(elevator.aboveIntake),
                new MovePCM(arm, -227), //-227
                new WaitUntilCommand(() -> elevator.getPosition() > 37 && arm.getPosition() < -226),
                new ReleaseGripperCommand(gripper).withTimeout(0.5)
            )
            )
        );
    }
}