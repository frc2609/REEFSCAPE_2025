// package frc.robot.commands.gripper;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Gripper;

// public class MoveGripperToPosition extends Command {
//     private final Gripper gripper;
//     private final Supplier<Double> targetPositionSupplier;

//     public MoveGripperToPosition(Gripper gripper, Supplier<Double> targetPositionSupplier) {
//         this.gripper = gripper;
//         this.targetPositionSupplier = targetPositionSupplier;
//         addRequirements(gripper);
//     }

//     @Override
//     public void execute() {
//         double currentTarget = targetPositionSupplier.get();
//         gripper.goToPosition(currentTarget);
//     }

//     // @Override
//     // public boolean isFinished() {
//     //     return arm.atPosition();
//     // }

//     @Override
//     public void end(boolean interrupted) {
//         gripper.stop();
//     }
// } 