package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.gripper.StopGripperCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class CoralHandOff extends SequentialCommandGroup{
    public CoralHandOff(Elevator elevator, Arm arm, Gripper gripper ) {
        addRequirements(elevator, arm, gripper);
        addCommands(
            // new PrintCommand("In CoralHandOff"),
            new MovePCM(arm, 20),
            new WaitUntilCommand(arm.aboveCoral),
            new MovePCM(elevator, 1),
            new WaitUntilCommand(elevator.handoff),
            new MovePCM(arm, -20),
            new GripCommand(gripper),
            new WaitCommand(0.5),
            new StopGripperCommand(gripper)//,
            // new PrintCommand("Done CoralHandOff")
        );
    }
}
