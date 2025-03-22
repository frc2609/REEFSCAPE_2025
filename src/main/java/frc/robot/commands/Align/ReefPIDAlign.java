package frc.robot.commands.Align;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefPIDAlign extends SequentialCommandGroup {
    public ReefPIDAlign(CommandSwerveDrivetrain drivetrain){
        addCommands(
            new PIDAlign(drivetrain), 
            new PIDFineAlign(true, drivetrain)
        );
    }

    
}
