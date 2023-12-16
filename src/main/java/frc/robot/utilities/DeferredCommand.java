package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class DeferredCommand extends CommandBase {
  private Supplier<Command> commandSupplier;
  private Command currentCommand;

  public DeferredCommand(Supplier<Command> commandSupplier, Subsystem... reqirements) {
    this.commandSupplier = commandSupplier;

    addRequirements(reqirements);
  }

  @Override
  public void initialize() {
    currentCommand = commandSupplier.get();
    currentCommand.initialize();
  }

  @Override
  public void execute() {
    currentCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    currentCommand.end(interrupted);
    currentCommand = null;
  }

  @Override
  public boolean isFinished() {
    return currentCommand.isFinished();
  }
}
