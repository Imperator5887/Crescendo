package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.NamedCommands;  

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.shootingConstraints;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.PivoteoCommand;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.swerve.autoAlign;
import frc.robot.subsystems.Mecanismos.Pivoteo;

public class subroutines {

    Pivoteo arm;

    public subroutines(){
        
        arm = Pivoteo.getInstance();

    }

    public enum ShooterState {
        SHOOT_FROM_SUBWOOFER, SHOOT_BEHIND_SUBWOOFER, SHOOT_FROM_INITIAL_NOTE
      }
    
    
    public static shootingConstraints getShootingState(ShooterState shooterState){
       
        shootingConstraints constraints;
        switch (shooterState) {
            case SHOOT_FROM_SUBWOOFER:
                constraints = new shootingConstraints(
                    0.7, 
                    0.061, 
                    0.95);
                break;
            
            case SHOOT_BEHIND_SUBWOOFER:
                constraints = new shootingConstraints(
                    0.9, 0.085, 1.2);
                break;

            case SHOOT_FROM_INITIAL_NOTE:
                constraints = new shootingConstraints(
                    0.9,
                     0.09, 
                     1.3);
                    break;
        
            default:
                constraints = new shootingConstraints(
                    0.7, 
                    0.061, 
                    0.95);
                break;

        }
        return constraints;

      }

     public static Command shootWithDelay(shootingConstraints constraints) {
        return new SequentialCommandGroup(
            new ShooterButtonCmd(constraints.shooterVelocity).withTimeout(constraints.delayTime), // DELAY CHANGED FROM 1s to 0.8s
            new ParallelCommandGroup(
                new ShooterButtonCmd(constraints.shooterVelocity),
                new IntakeButtonCmd(-0.5))
                ).withTimeout(2);
    
    }

    public static Command shootToAmp() {
        return new ParallelCommandGroup(
                new ShooterButtonCmd(0.225),
                new IntakeButtonCmd(-0.25)
                ).withTimeout(1.2);
    
    }

    public static Command lowArm() {
        return new SequentialCommandGroup(
            new PivoteoCommand(0.26),
            new PivoteoCommand(0.061)
        );     
    }

    public static Command lowArmAndShoot(shootingConstraints constraints) {
        return new SequentialCommandGroup(
            new PivoteoCommand(0.26),
            new ParallelCommandGroup(
                new PivoteoCommand(constraints.pivotPosition),
                shootWithDelay(constraints)
            )
        ).withTimeout(2.3);     
    }

   
    /*public static Command alignAndShit(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new autoAlign(aprilTag.constraints),
                new 
            )

        )
    }*/

    
}
