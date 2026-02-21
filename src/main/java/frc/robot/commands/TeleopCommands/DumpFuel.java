package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class DumpFuel extends SequentialCommandGroup {
    
    public DumpFuel(AngularVelocity speed, AngularVelocity transitionSpeed) {
        addCommands(
            new StowIntake().withTimeout(0.75),
            new ParallelCommandGroup(
                RobotContainer.intakeStaticRoller.setVelocity(speed),
                RobotContainer.intakeMainRoller.setVelocity(speed),
                RobotContainer.transition.setVelocity(transitionSpeed)
        ));
    }
}
