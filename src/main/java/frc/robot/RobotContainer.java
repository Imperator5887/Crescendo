/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PS4OIConstants;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.commands.Mecanismos.ArmVelocityCommand;
import frc.robot.commands.Mecanismos.ClimbCommand;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.PhotonLLCommand;
import frc.robot.commands.Mecanismos.PivoteoCommand;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.Mecanismos.setPivotVelocity;
import frc.robot.commands.hybrid.autos;
import frc.robot.commands.hybrid.subroutines;
import frc.robot.commands.hybrid.subroutines.ShooterState;
import frc.robot.commands.swerve.autoAlign;
import frc.robot.commands.swerve.swerveDriveComando;
import frc.robot.commands.swerve.swervePrecisionCommand;
import frc.robot.subsystems.Mecanismos.ClimberSubsystem;
import frc.robot.subsystems.Mecanismos.IntakeSubsystem;
import frc.robot.subsystems.Mecanismos.Pivoteo;
import frc.robot.subsystems.Mecanismos.ShooterSubsystem;
import frc.robot.subsystems.swerve.swerveSusbsystem;
import frc.robot.subsystems.vision.PhotonLL;

public class RobotContainer {

    private swerveSusbsystem swerve;
    private PhotonLL photoncamera;
    private Pivoteo arm;
    private ClimberSubsystem climber;
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;


    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    public RobotContainer(){

        
        
        swerve = swerveSusbsystem.getInstance();
        photoncamera = PhotonLL.getInstance();
        arm = Pivoteo.getInstance();
        climber = ClimberSubsystem.getInstance();
        m_intakeSubsystem = IntakeSubsystem.getInstance();
        m_shooterSubsystem = ShooterSubsystem.getInstance();


        // "save" a command in order to call it within an event marker.

        /* 
        swerveSubsystem.setDefaultCommand(new swerveDriveComando(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                true
                ));

        */
        swerve.setDefaultCommand(new swerveDriveComando(
                    swerve,
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true
                    ));

              photoncamera.setDefaultCommand(new PhotonLLCommand());

              //pivoteo deffault command:
              arm.setDefaultCommand(new ArmVelocityCommand(0));

        // Intake:
        m_intakeSubsystem.setDefaultCommand(
            new IntakeButtonCmd(0)//
        );

        // shooter:
        m_shooterSubsystem.setDefaultCommand(
            new ShooterButtonCmd(0)//
        );

        climber.setDefaultCommand(
            new ClimbCommand(false)
        );

        

                
              
        configureButtonBindings();

         
         
    }

   

    
    private void configureButtonBindings() {

       
         
       // ========================== DRIVER JOYSTICKS  ========================================//

        // LOWER VELOCITY:
       new JoystickButton(driverJoytick, Constants.PS4OIConstants.topLeft).whileFalse(
        new swervePrecisionCommand(swerve, 
        () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true)
       );

        // INCREASE VELOCITY:
       new JoystickButton(driverJoytick, Constants.PS4OIConstants.topRight).whileFalse(
        new swerveDriveComando(swerve, 
        () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true)
       );

        // RESEAT HEADING:
        new JoystickButton(driverJoytick, Constants.PS4OIConstants.triangle).whileFalse(
            new InstantCommand(
                () -> swerve.resetHeading()
            )
        );

        // ALIGN TO APRILTAG:
        new JoystickButton(driverJoytick, Constants.PS4OIConstants.square).whileTrue(new autoAlign(aprilTag.constraints));


       // =============================== ÁNGULOS PIVOTEO ==================================== //
        
        // BAJAR BRAZO
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.square).whileTrue(new SequentialCommandGroup(
            new PivoteoCommand(0.26),
            new PivoteoCommand(0.061)
        )); // 44°
       
        // ÁNGULO SHOOT:
        new JoystickButton(placerJoystick,Constants.PS4OIConstants.joystickIzq).whileTrue(new PivoteoCommand(0.075));// 44°

        // ÁNGULO TRAP/SHOOTEAR DE LEJOS
        new JoystickButton(placerJoystick,Constants.PS4OIConstants.joystickDer).whileTrue(new PivoteoCommand(0.061)); // 35°

        // ÁNGULO TRAP/ SHOOTEAR DE LEJOS:
       new JoystickButton(placerJoystick,Constants.PS4OIConstants.triangle).whileTrue(new PivoteoCommand(0.122)); // 44°

        // ÁNGULO CLIMBER:
        //new JoystickButton(placerJoystick,12).whileTrue(new PivoteoCommand(0.43)); // ir tantito p atrás
        new JoystickButton(placerJoystick,Constants.PS4OIConstants.joystickDer).whileFalse(new PivoteoCommand(0.51)); // 90 grados (empujón)

        
        // Tiro al AMP (Boton de PS)
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.PSButton).whileTrue(subroutines.shootToAmp()); 

       // =============================== NOTE SCORING ====================================== //

        // shoot sin delay:
        //new JoystickButton(placerJoystick, 5).whileTrue(new ShooterButtonCmd(-0.75)); // 

        // SHOOT
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.topRight).whileTrue(subroutines.shootWithDelay(subroutines.getShootingState(ShooterState.SHOOT_FROM_SUBWOOFER)));

        // recoger
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.cross).whileTrue(new IntakeButtonCmd(-0.5, true));// ********true
        
        // escupir
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.circle).whileTrue(new IntakeButtonCmd(0.5));

        // climber:
        new POVButton(placerJoystick, 90).whileTrue(new ClimbCommand(true));

         //================================== APRIL TAG: ==============================================//
       //new JoystickButton(driverJoytick, 5).whileTrue(new autoAlign());
       //new JoystickButton(driverJoytick, 2).whileTrue(new autoAlign(limelightConstants.noteOffsets.offsets));

       // =============================  SHOOTING POSITIONS TBC ===================================== //
       // --> SUBWOOFER <----
       //new JoystickButton(placerJoystick, 10).whileTrue(new PivoteoCommand(0.061)); // 35°

       // --> ROBOT STARTING ZONE <---
       //new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.0820)); // x

    
        // ======================================== X-BOX PLACER JOYSTICKS ===========================//
        // MECHANISMS
        //new JoystickButton(placerJoystick,9).whileTrue(new PivoteoCommand(0.075)); // 44°

       //new JoystickButton(placerJoystick,10).whileTrue(new PivoteoCommand(0.40)); // 

        
    }

    public Command getAutonomousCommand() {
       
       return autos.test_papaya();

    }


}
