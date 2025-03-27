package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Intake.flop.RetractFlopCommand;
import frc.robot.commands.Intake.roll.StopRollCommand;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.IntakeRoll;

public class RetractIntakeCommand extends ParallelCommandGroup{
    public RetractIntakeCommand(IntakeFlop intakeFlop, IntakeRoll intakeRoll) {
        addCommands(
            // new PrintCommand("In RetractIntakeCommand"),
            new RetractFlopCommand(intakeFlop),
            new StopRollCommand(intakeRoll)//,
            // new PrintCommand("Done RetractIntakeCommand")
        );
    }
    
}
