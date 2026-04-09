from frc.tools import Component
import asyncio

class ManuelController(Component):
    # --- Subsystem References ---
    # These must match the variable names in your robot.py
    __feeder: object
    __indexer: object
    __shooter: object
    __swerve: object
    
    # --- Input Mapping ---
    # These are the button/axis objects from your configuration
    __toggle_manuel: object      # Y Button
    __manuelshooter: object      # A Button
    __manuelsouffleur: object    # B Button
    __manuelshooterspeed: object  # Left Joystick Y
    __manuelspinturret: object    # Triggers
    __souffleuse_button: object   # Original Auto Button

    # --- State Variables ---
    __is_manuel_mode: bool = False
    __requested_mode: str = "DEFAULT"

    def __input_loop__(self):
        """Processes driver inputs continuously."""
        while True:
            # Toggle Manuel Mode (Second Driver) via Y button
            if self.__toggle_manuel.get_button_down():
                self.__is_manuel_mode = not self.__is_manuel_mode
            
            yield

    def update(self):
        """Main control logic loop executed by MagicBot."""
        
        if self.__is_manuel_mode:
            # 1. Manuel Shooter and Pre-Feeder (A Button)
            manuel_shooter_state = self.__manuelshooter.get()
            self.__shooter.set_shooting(manuel_shooter_state)

            # 2. Manuel Souffleur and Cassette (B Button)
            manuel_souffleur_state = self.__manuelsouffleur.get()
            self.__feeder.set_feeding(manuel_souffleur_state)

            # 3. Indexer logic: Runs when either manual button is pressed
            self.__indexer.set_indexing(manuel_shooter_state or manuel_souffleur_state)

            # 4. Manuel Shooter Speed (Left Joystick Y)
            shooter_speed = self.__manuelshooterspeed.get()
            self.__shooter.set_shooter_speed(shooter_speed)

            # 5. Manuel Spin Turret (Triggers/Joystick)
            turret_spin = self.__manuelspinturret.get()
            if abs(turret_spin) > 0:
                current_heading = self.__shooter.current_heading()
                # Incremental adjustment: 0.005 is a speed multiplier
                self.__shooter.set_target_heading(current_heading + turret_spin * 0.005, False)

            # Adjust drive speed during manual override for safety
            if manuel_shooter_state or manuel_souffleur_state:
                self.__swerve.set_speed(0.5)
            else:
                self.__swerve.set_speed(1)

        else:
            # --- Original Automatic Control Logic ---
            souffleuse_state = self.__souffleuse_button.get()
            self.__feeder.set_feeding(souffleuse_state)
            self.__indexer.set_indexing(souffleuse_state)
            self.__shooter.set_shooting(souffleuse_state)

            if souffleuse_state:
                self.__swerve.set_speed(0.5)
            else:
                self.__swerve.set_speed(1)