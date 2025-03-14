package frc.robot.commands.Align;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LeftAlign extends SequentialCommandGroup{
   public LeftAlign(CommandSwerveDrivetrain swerveDrivetrain){
       addCommands(
        new PIDAlign(swerveDrivetrain),
        new PIDFineAlign(true, swerveDrivetrain)
       );
       
   }


}
