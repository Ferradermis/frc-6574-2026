package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import frc.robot.commands.TeleopCommands.StowIntake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class IntakeAutoStop extends SequentialCommandGroup {
    
    public IntakeAutoStop() {
        addCommands(
            RobotContainer.intakeMainRoller.setVelocity(RPM.of(00)),
            RobotContainer.transition.setVelocity(RPM.of(00)),
            new ParallelCommandGroup(
                new StowIntake()
                
        ));
    }
}
