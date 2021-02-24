package frc.robot;

import java.io.BufferedReader;

import java.io.FileReader;
import java.io.FilterInputStream;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.HashMap;

public class Configrun {
    private static HashMap<String, String> keys = new HashMap<String, String>();

    static {
        // System.err.println("in config");
        // System.err.println("in config");
        System.err.println("in config");
        loadconfig();

    }

    public static int get(int defaultvalue, String key) {
      if (keys.get(key) != null) {
            //System.out.println("Run");
            return Integer.valueOf(keys.get(key));

            // return(keys.get(key));
        } else {
            System.err.println("No such value/type1");
            return defaultvalue;
        }
    }

    public static double get(double defaultvalue, String key) {
        if (keys.get(key) != null) {
            return Double.valueOf(keys.get(key));
            // return(keys.get(key));
        } else {
            System.err.println("No such value/type2");
            return defaultvalue;
        }

    }

    public static boolean get(boolean defaultvalue, String key) {
        if (keys.get(key) != null) {
            return Boolean.valueOf(keys.get(key));
            // return(keys.get(key));
        } else {
            System.err.println("No such value/type");
            return defaultvalue;
        }

    }

    public static void loadconfig() {
        BufferedReader reader;

        try {
            // int linenumber=0;
            reader = new BufferedReader(new FileReader("/home/lvuser/deploy/config.txt"));
            String line;
            //System.out.println("in load config");
            // reader.readLine();
            // System.out.println(line);
            // for (int run2 = 0; run2 <= line.length() - 1;) {
            // int run2=0;
            // System.out.println(line);
            // if (line.charAt(run2) == '#'){
            while ((line = reader.readLine()) != null) {
                // System.out.println("trim config");
                int linelength = line.trim().length();

                // line = reader.readLine();
                // System.out.println(linelength);
                if (linelength > 0) {
                    if (!line.startsWith("#")) {
                        // if (!line.startsWith(" ")) {

                        // System.out.println(line);
                        String[] array = line.split("=");
                        // System.out.println(array[0]);
                        // System.out.println(array[1]);
                        //System.out.println("hashmap config");
                        
                        keys.put(array[0].trim(), array[1].trim());
                    }
                }

                /*
                 * else { line = reader.readLine(); //}
                 */

            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
            System.err.println("in config");
            System.err.println("in config");
            System.err.println("in config");
        }
    }
}
