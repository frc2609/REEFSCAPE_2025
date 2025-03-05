package frc.robot.commands.Intake.flop;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlop;

public class DeployFlopCommand extends Command {
    private final IntakeFlop intakeFlop;
    private final double target = 23.0;

    public DeployFlopCommand(IntakeFlop intakeFlop) {
        this.intakeFlop = intakeFlop;
        addRequirements(intakeFlop);
    }

    @Override
    public void execute() {
        intakeFlop.setNeutralMode(NeutralModeValue.Coast);
        intakeFlop.goToPosition(target);
    }

    // @Override
    // public boolean isFinished() {
    //     // return false;
    //    return true;
    // }

    @Override
    public void end(boolean interrupted) {
        // Stop intakeFlop when command ends
        intakeFlop.stop();
    }
}