package frc.robot.commands.Intake.flop;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.IntakeFlop;

public class DeployFlopCommand extends SequentialCommandGroup {
    private final double target = 95;

    public DeployFlopCommand(IntakeFlop intakeFlop) {
        addCommands(
            new MovePCM(intakeFlop,target),
            new SequentialCommandGroup(
                new WaitUntilCommand(intakeFlop.deployedTrigger),
                new InstantCommand(() -> intakeFlop.stop())
            ),
            new SequentialCommandGroup(
                new WaitUntilCommand(intakeFlop.coralTriggrt),
                new RetractFlopCommand(intakeFlop)
            )
        );
    }
}