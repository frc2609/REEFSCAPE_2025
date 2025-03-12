package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ScoreL2CommandWithWait extends SequentialCommandGroup {
    public ScoreL2CommandWithWait(Elevator elevator, Arm arm, Gripper gripper, CommandXboxController controller) {
            addCommands(

                new MovePCM(arm, -227),
                new WaitUntilCommand(arm.aboveIntake),
                new MovePCM(elevator, 0),
                
                new WaitUntilCommand(() -> 
                    controller.a().getAsBoolean() || // A to release
                    controller.b().getAsBoolean()    // B to stow
                ),
                
                new ParallelCommandGroup(
                    new ConditionalCommand(
                        new ReleaseGripperCommand(gripper),  // If A was pressed
                        new StowCommand(elevator, arm),      // If B was pressed
                        () -> controller.a().getAsBoolean()
                    )
                )
            );
    }
} 