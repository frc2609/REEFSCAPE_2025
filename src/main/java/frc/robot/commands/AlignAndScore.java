// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.Align.PIDFineAlign;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class AlignAndScore extends SequentialCommandGroup {
//     public AlignAndScore(Command scoreCommand, CommandSwerveDrivetrain drivetrain, Trigger rightConfirm, Trigger leftConfirm) {
//         addCommands(
//             new WaitUntilCommand(rightConfirm.or(leftConfirm)),
//             new ConditionalCommand(
//                 new PIDFineAlign(true, drivetrain), 
//                 new PIDFineAlign(false, drivetrain), 
//                 leftConfirm
//             ),
//             new ConditionalCommand(
//                 scoreCommand, 
//                 new InstantCommand(), 
//                 PIDFineAlign.aligned
//             )
//         );
//     }
    
// }
