package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.flop.DeployFlopCommand;
import frc.robot.commands.Intake.roll.IntakeRollCommand;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.IntakeRoll;

public class DeployIntakeCommand extends ParallelCommandGroup {
    public DeployIntakeCommand(IntakeFlop intakeFlop, IntakeRoll intakeRoll) {
        addCommands(
            new ConditionalCommand(
                new InstantCommand(), 
                new ParallelCommandGroup(
                    new DeployFlopCommand(intakeFlop),
                    new IntakeRollCommand(intakeRoll)
                ), 
                intakeFlop.coralTrigger)
        );
    }
}
