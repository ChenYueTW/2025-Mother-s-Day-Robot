package frc.robot.lib.data;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

public class TrajectoryGenerator {
    private static final Map<String, double[]> coordinates = new LinkedHashMap<>() {{
        put("S1", new double[]{8.02, 7.25, 0});
        put("S2", new double[]{8.02, 6.16, 0});
        put("S3", new double[]{8.02, 5.07, 0});
        put("S4", new double[]{8.02, 3, 0});
        put("S5", new double[]{8.02, 1.9, 0});
        put("S6", new double[]{8.02, 0.8, 0});
        put("A left", new double[]{3.22, 4.19, 180});
        put("A right", new double[]{3.22, 3.86, 180});
        put("B right", new double[]{3.7, 5.05, 120});
        put("B left", new double[]{3.99, 5.22, 120});
        put("C right", new double[]{4.99, 5.22, 60});
        put("C left", new double[]{5.28, 5.05, 60});
        put("D left", new double[]{5.75, 4.19, 0});
        put("D right", new double[]{5.75, 3.86, 0});
        put("E left", new double[]{5.28, 3, -60});
        put("E right", new double[]{4.99, 2.83, -60});
        put("F left", new double[]{3.99, 2.83, -120});
        put("F right", new double[]{3.7, 3, -120});
    }};

    public static void main(String[] args) {
        SwingUtilities.invokeLater(TrajectoryGenerator::createAndShowGUI);
    }

    private static void createAndShowGUI() {
        JFrame frame = new JFrame("Trajectory Generator");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(400, 200);
        frame.setLayout(new GridLayout(4, 2));

        JComboBox<String> startBox = new JComboBox<>(new String[]{"S1", "S2", "S3", "S4", "S5", "S6"});
        JComboBox<String> pointBox = new JComboBox<>(new String[]{"A", "B", "C", "D", "E", "F"});
        JComboBox<String> sideBox = new JComboBox<>(new String[]{"left", "right"});
        JButton generateButton = new JButton("Generate");

        frame.add(new JLabel("Start Position:"));
        frame.add(startBox);
        frame.add(new JLabel("Target Point:"));
        frame.add(pointBox);
        frame.add(new JLabel("Side:"));
        frame.add(sideBox);
        frame.add(generateButton);

        generateButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String start = (String) startBox.getSelectedItem();
                String point = (String) pointBox.getSelectedItem();
                String side = (String) sideBox.getSelectedItem();
                generateTrajectory(start, point, side);
            }
        });

        frame.setVisible(true);
    }

    private static void generateTrajectory(String start, String point, String side) {
        String key1 = start;
        String key2 = point + " " + side;

        if (!coordinates.containsKey(key1) || !coordinates.containsKey(key2)) {
            JOptionPane.showMessageDialog(null, "Invalid selection.");
            return;
        }

        double[] startCoords = coordinates.get(key1);
        double[] endCoords = coordinates.get(key2);
        System.out.println(startCoords.toString());

        String fileName = start + "_" + point + side.toUpperCase().charAt(0) + ".traj";
        File directory = new File("src/main/java/frc/robot/lib/data/pathGenerate");
        if (!directory.exists()) {
            directory.mkdirs();
        }
        File file = new File(directory, fileName);

        Map<String, Object> trajData = new LinkedHashMap<>();
        trajData.put("name", fileName.replace(".traj", ""));
        trajData.put("version", 1);
        trajData.put("snapshot", createSnapshot());
        trajData.put("params", createParams(startCoords, endCoords));
        trajData.put("trajectory", createTrajectory());
        trajData.put("events", new Object[]{});

        Gson gson = new GsonBuilder().create();
        String jsonContent = gson.toJson(trajData);

        try (FileWriter writer = new FileWriter(file)) {
            writer.write(jsonContent);
            JOptionPane.showMessageDialog(null, "File generated: " + file.getAbsolutePath());
        } catch (IOException ex) {
            JOptionPane.showMessageDialog(null, "Error writing file.");
        }
    }

    private static Map<String, Object> createSnapshot() {
        Map<String, Object> snapshot = new LinkedHashMap<>();
        snapshot.put("waypoints", new Object[]{});
        snapshot.put("constraints", new Object[]{});
        snapshot.put("targetDt", 0.05);
        return snapshot;
    }

    private static Map<String, Object> createParams(double[] startCoords, double[] endCoords) {
        Map<String, Object> params = new LinkedHashMap<>();
        params.put("waypoints", new Object[]{
                createWaypoint(startCoords),
                createWaypoint(endCoords)
        });
        params.put("constraints", new Object[]{
                createConstraint("first", "null", "StopPoint"),
                createConstraint("last", "null", "StopPoint"),
                createKeepInRectangleConstraint()
        });
        params.put("targetDt", createTargetDt());
        return params;
    }

    private static Map<String, Object> createTrajectory() {
        Map<String, Object> trajectory = new LinkedHashMap<>();
        trajectory.put("sampleType", "null");
        trajectory.put("waypoints", new Object[]{});
        trajectory.put("samples", new Object[]{});
        trajectory.put("splits", new Object[]{});
        return trajectory;
    }

    private static Map<String, Object> createWaypoint(double[] coords) {
        Map<String, Object> waypoint = new LinkedHashMap<>();
        waypoint.put("x", Map.of("exp", coords[0] + " m", "val", coords[0]));
        waypoint.put("y", Map.of("exp", coords[1] + " m", "val", coords[1]));
        waypoint.put("heading", Map.of("exp", coords[2] + " deg", "val", Math.toRadians(coords[2])));
        waypoint.put("intervals", 40);
        waypoint.put("split", false);
        waypoint.put("fixTranslation", true);
        waypoint.put("fixHeading", true);
        waypoint.put("overrideInterals", false);
        return waypoint;
    }

    private static Map<String, Object> createConstraint(String from, String to, String type) {
        Map<String, Object> constraints = new LinkedHashMap<>();
        constraints.put("from", from);
        constraints.put("to", to);
        constraints.put("data", Map.of("type", type, "props", new LinkedHashMap<>()));
        constraints.put("enabled", true);
    
        return constraints;
    }

    private static Map<String, Object> createTargetDt() {
        Map<String, Object> targetDt = new LinkedHashMap<>();
        targetDt.put("exp", "0.05 s");
        targetDt.put("val", 0.05);
        return targetDt;
    }

    private static Map<String, Object> createKeepInRectangleConstraint() {
        return createConstraint("first", "last", "KeepInRectangle");
    }
}
