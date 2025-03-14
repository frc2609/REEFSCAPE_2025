package frc.robot.commands;

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

public class ScoreL4Command extends ParallelCommandGroup{
    public ScoreL4Command(Elevator elevator, Arm arm, Gripper gripper, Trigger shootTrigger){
        addCommands(
            new MovePCM(elevator, 38), //39
            new SequentialCommandGroup(
                new WaitUntilCommand(elevator.aboveIntake),
                new MovePCM(arm, -227), //-227
                new WaitUntilCommand(shootTrigger),
                new ReleaseGripperCommand(gripper),
                new WaitCommand(0.5)
            )
        );
    }
}