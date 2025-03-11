package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class PickAlgaeL3Command extends ParallelCommandGroup{
    public PickAlgaeL3Command(Elevator elevator, Arm arm, Gripper gripper) {
        addCommands(
            new MovePCM(elevator, 20),
            new MovePCM(arm, 45),
            new GripCommand(gripper)
        );
    }
}
