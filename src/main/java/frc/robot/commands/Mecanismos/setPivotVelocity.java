package frc.robot.commands.Mecanismos;

import frc.robot.subsystems.Mecanismos.Pivoteo;
import edu.wpi.first.wpilibj2.command.Command;

public class setPivotVelocity extends Command {
  private final Pivoteo arm;

    private final double velocity;     
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public setPivotVelocity(double velocity) {
    arm = Pivoteo.getInstance();
    this.velocity = velocity;
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("ENDED SUCCESSFULLY");
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

