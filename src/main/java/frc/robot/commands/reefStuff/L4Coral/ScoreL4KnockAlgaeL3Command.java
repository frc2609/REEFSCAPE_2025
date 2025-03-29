package frc.robot.commands.reefStuff.L4Coral;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class ScoreL4KnockAlgaeL3Command extends SequentialCommandGroup{

    public ScoreL4KnockAlgaeL3Command(Elevator elevator, Arm arm, Gripper gripper, Trigger shootTrigger, Trigger confirmTrigger, Trigger confirm2Trigger){
        addCommands(
            new ParallelCommandGroup(

            new MovePCM(elevator, 39.5), //39
            new SequentialCommandGroup(
                new WaitUntilCommand(elevator.aboveIntake),
                new WaitUntilCommand(confirm2Trigger.or(confirmTrigger)),
                new MovePCM(arm, -227), //-227
                new WaitUntilCommand(shootTrigger),
                new ReleaseGripperCommand(gripper),
                new WaitCommand(0.5)

                )

            ),
            new MovePCM(elevator, 23),
            new ParallelCommandGroup(
            new GripCommand(gripper),
            new MovePCM(arm, 60),
            new WaitUntilCommand(shootTrigger)

            ),
            Commands.idle(elevator, arm, gripper)
        );
    }
}