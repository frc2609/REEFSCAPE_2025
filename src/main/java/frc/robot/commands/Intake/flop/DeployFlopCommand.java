package frc.robot.commands.Intake.flop;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.RetractIntakeCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.IntakeFlop;

public class DeployFlopCommand extends SequentialCommandGroup {
    private final double target = 90;

    public DeployFlopCommand(IntakeFlop intakeFlop) {
        addCommands(
            new InstantCommand(() -> intakeFlop.enablePositionControl()),
            new MovePCM(intakeFlop,target),
            new WaitUntilCommand(intakeFlop.deployedTrigger),
            new InstantCommand(() -> intakeFlop.disablePositionControl()),
            new InstantCommand(() -> intakeFlop.setNeutralMode(NeutralModeValue.Coast))
        );
    }
}