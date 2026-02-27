package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class IntakeAuto extends SequentialCommandGroup {
    
    public IntakeAuto(AngularVelocity speed, AngularVelocity transitionSpeed) {
        addCommands(
            RobotContainer.fuelRamp.setAngle(Degrees.of(-18)).withTimeout(0.25),
            RobotContainer.intakePivot.setAngle(Degrees.of(-8.5)).withTimeout(0.5),
            new ParallelCommandGroup(
                RobotContainer.intakeMainRoller.setVelocity(speed),
                RobotContainer.transition.setVelocity(transitionSpeed)
        ));
    }
}
