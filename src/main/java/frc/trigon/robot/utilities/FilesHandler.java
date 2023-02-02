package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class FilesHandler {
    public static final String DEPLOY_PATH = Filesystem.getDeployDirectory().getPath() + "/";

    /**
     * Deletes the given file.
     *
     * @param absolutePath the file's absolute path
     * @throws IOException if the method failed to delete the specified file
     */
    public static void deleteFile(String absolutePath) throws IOException {
        File file = new File(absolutePath);
        if(!file.delete())
            throw new IOException("Failed to delete the file \"" + absolutePath + "\".");
    }

    /**
     * Writes the given string to a file.
     *
     * @param absolutePath the file's absolute path
     * @param str          the string to write to the file
     * @throws IOException if the method failed to write the string to the file
     */
    public static void writeStringToFile(String absolutePath, String str) throws IOException {
        FileWriter fileWriter = new FileWriter(absolutePath);
        fileWriter.write(str);
        fileWriter.close();
    }

    /**
     * Renames a file.
     *
     * @param absolutePath the file's absolute path
     * @param newName      the new desired name
     * @throws IOException if the method failed to rename the file
     */
    public static void renameFile(String absolutePath, String newName) throws IOException {
        File file = new File(absolutePath);
        String newAbsolutePath = extractPathFromAbsolutePath(absolutePath) + newName;
        if(!file.renameTo(new File(newAbsolutePath)))
            throw new IOException("Failed to rename file " + absolutePath + " to " + newName);
    }

    /**
     * Creates a file using a safe method of writing.
     * This method will write the string to a temporary file,
     * rename the existing file to its name.bak,
     * rename the temporary file to the desired name,
     * and delete the .bak file.
     *
     * @param absolutePath the file's absolute path
     * @param str          the string to write to the file
     * @throws IOException if the method failed to safe write the file
     */
    public static void safeWrite(String absolutePath, String str) throws IOException {
        if(fileExists(absolutePath + ".tmp"))
            deleteFile(absolutePath + ".tmp");
        writeStringToFile(absolutePath + ".tmp", str);
        if(fileExists(absolutePath))
            renameFile(absolutePath, extractFileNameFromAbsolutePath(absolutePath) + ".bak");
        renameFile(absolutePath + ".tmp", extractFileNameFromAbsolutePath(absolutePath));
        if(fileExists(absolutePath + ".bak"))
            deleteFile(absolutePath + ".bak");
    }

    /**
     * Reads a file and returns its content as a string.
     * If the file does not exist, it will check for a .tmp file.
     *
     * @param absolutePath the file's absolute path
     * @return the file contents
     * @throws IOException if the method failed to read the file
     */
    public static String readFile(String absolutePath) throws IOException {
        String newPath = !fileExists(absolutePath) && fileExists(absolutePath + ".tmp") ?
                         absolutePath + ".tmp" :
                         absolutePath;
        return Files.readString(Path.of(newPath));
    }

    private static boolean fileExists(String absolutePath) {
        return new File(absolutePath).exists();
    }

    private static String extractPathFromAbsolutePath(String absolutePath) {
        return absolutePath.substring(0, absolutePath.lastIndexOf("/") + 1);
    }

    private static String extractFileNameFromAbsolutePath(String absolutePath) {
        return absolutePath.substring(absolutePath.lastIndexOf("/") + 1);
    }
}
