package frc.robot.commands.Align;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RightAlign extends SequentialCommandGroup {

   public RightAlign(CommandSwerveDrivetrain swerveDrivetrain){
       addCommands(
        new PIDAlign(swerveDrivetrain),
        new PIDFineAlign(false, swerveDrivetrain)

       );
       
   }


}


