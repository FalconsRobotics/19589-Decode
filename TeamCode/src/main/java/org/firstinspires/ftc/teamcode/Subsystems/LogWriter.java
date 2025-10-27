package org.firstinspires.ftc.teamcode.Subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class LogWriter {
    private final String filename;
    private final String directoryPath = "/sdcard/FIRST/logs";
    private final File logFile;

    /**
     * Initializes the LogWriter.
     * @param filename The name of the file to write to (e.g., "drive_data.txt").
     */
    public LogWriter(String filename) {
        this.filename = filename;
        // 1. Ensure the log directory exists
        new File(directoryPath).mkdirs();
        // 2. Define the full path for the log file
        this.logFile = new File(directoryPath, filename);
    }

    /**
     * Writes a single line of data to the file, appending it to the end.
     * Includes a timestamp for context.
     * @param data The string content to write.
     */
    public void write(String data) {
        // We call the internal method with 'true' to ensure append mode
        writeInternal(data, true);
    }

    /**
     * Writes data to the file, either appending or overwriting based on the boolean.
     * This is useful for clearing the file on initialization (overwrite) or logging (append).
     * @param data The string content to write.
     * @param append If true, append to the end. If false, overwrite the file.
     */
    public void write(String data, boolean append) {
        writeInternal(data, append);
    }

    private void writeInternal(String data, boolean append) {
        // Use try-with-resources to ensure the stream is automatically closed
        try (FileWriter fileWriter = new FileWriter(logFile, append);
             BufferedWriter bufferedWriter = new BufferedWriter(fileWriter)) {

            // Prepend a timestamp only if appending data
            if (append) {
                String timestamp = String.format("[%d] ", System.currentTimeMillis());
                bufferedWriter.write(timestamp + data);
                bufferedWriter.newLine();
            } else {
                // If overwriting, just write the data
                bufferedWriter.write(data);
            }

        } catch (IOException e) {
            // In a real OpMode, you'd send this error to Telemetry
            System.err.println("File I/O Error: Could not write to " + logFile.getAbsolutePath());
            e.printStackTrace();
        }
    }
}
