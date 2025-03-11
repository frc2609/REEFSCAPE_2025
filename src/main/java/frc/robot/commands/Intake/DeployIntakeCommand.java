package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.flop.DeployFlopCommand;
import frc.robot.commands.Intake.roll.IntakeRollCommand;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.IntakeRoll;

public class DeployIntakeCommand extends ParallelCommandGroup {
    public DeployIntakeCommand(IntakeFlop intakeFlop, IntakeRoll intakeRoll) {
        addCommands(
            new DeployFlopCommand(intakeFlop),  // Deploys the intake (goes to position 23.0)
            new IntakeRollCommand(intakeRoll)          // Runs the roll at -0.5 speed
        );
    }
}
