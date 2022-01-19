package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Configrun {
    private static HashMap<String, String> keys = new HashMap<String, String>();
    private static NetworkTableEntry theRobot = Shuffleboard.getTab("TestValues").add("ConfigFile", "NoConfig")
            .getEntry();
    static {
        System.err.println("in config");
        loadconfig();
    }

    public static int get(int defaultvalue, String key) {
        if (keys.get(key) != null) {
            return Integer.valueOf(keys.get(key));
        } else {
            System.err.println("No such value/type1");
            return defaultvalue;
        }
    }

    public static double get(double defaultvalue, String key) {
        if (keys.get(key) != null) {
            return Double.valueOf(keys.get(key));
        } else {
            System.err.println("No such value/type2");
            return defaultvalue;
        }
    }

    public static boolean get(boolean defaultvalue, String key) {
        if (keys.get(key) != null) {
            return Boolean.valueOf(keys.get(key));
        } else {
            System.err.println("No such value/type");
            return defaultvalue;
        }
    }

    public static void loadconfig() {
        BufferedReader reader;

        try {
            // Checks to see what config to use based on a file manually put on the robot
            if (new File("/home/lvuser/proto").exists()) {
                reader = new BufferedReader(new FileReader("/home/lvuser/deploy/protoConfig.txt"));
                theRobot.setString("Proto");
            } else if (new File("/home/lvuser/dev").exists()) {
                reader = new BufferedReader(new FileReader("/home/lvuser/deploy/devConfig.txt"));
                theRobot.setString("Dev");
            } else {
                reader = new BufferedReader(new FileReader("/home/lvuser/deploy/compConfig.txt"));
                theRobot.setString("Comp");
            }
            String line;

            while ((line = reader.readLine()) != null) {
                int linelength = line.trim().length();

                if (linelength > 0 && !line.startsWith("#")) {
                    String[] array = line.split("=");
                    keys.put(array[0].trim(), array[1].trim());
                }
            }
            reader.close();
        }

        catch (IOException e) {
            e.printStackTrace();
            System.err.println("in config");
            System.err.println("in config");
            System.err.println("in config");
        }
    }
}