package frc.robot.lib.subsystems;

import java.util.ArrayList;

public class RobotProvider {
    private static final ArrayList<SubsystemBase> subsystemsProvider = new ArrayList<>();
    private static final ArrayList<CommandBase> commandsProvider = new ArrayList<>();

    public static void registerSubsystems(SubsystemBase subsystemBase) {
        subsystemsProvider.add(subsystemBase);
    }

    public static void registerCommands(CommandBase commandBase) {
        commandsProvider.add(commandBase);
    }

    public static void setAllDefaultCommands() {
        for (int i = 0; i < subsystemsProvider.size(); i++) {
            if (!subsystemsProvider.get(i).getChooser()) return;
            subsystemsProvider.get(i).setDefaultCommand(commandsProvider.get(i));
        }
    }
}
