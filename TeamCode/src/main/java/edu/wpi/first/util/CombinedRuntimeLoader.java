// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.util;

import android.annotation.SuppressLint;
import android.os.Build;

import androidx.annotation.RequiresApi;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Loads dynamic libraries for all platforms. */
public final class CombinedRuntimeLoader {
  private CombinedRuntimeLoader() {}

  private static boolean skipSetDllDirectory;

  /**
   * Flag to test not setting DLL directory for loading.
   *
   * @param skip Skip setting DLL directory
   */
  public static synchronized void setSkipSetDllDirectory(boolean skip) {
    skipSetDllDirectory = skip;
  }

  private static synchronized boolean getSkipSetDllDirectory() {
    return skipSetDllDirectory;
  }

  private static String extractionDirectory;

  /**
   * Returns library extraction directory.
   *
   * @return Library extraction directory.
   */
  public static synchronized String getExtractionDirectory() {
    return extractionDirectory;
  }

  private static synchronized void setExtractionDirectory(String directory) {
    extractionDirectory = directory;
  }

  private static String defaultExtractionRoot;

  /**
   * Gets the default extraction root location (~/.wpilib/nativecache) for use if
   * setExtractionDirectory is not set.
   *
   * @return The default extraction root location.
   */
  @RequiresApi(api = Build.VERSION_CODES.O)
  public static synchronized String getDefaultExtractionRoot() {
    if (defaultExtractionRoot != null) {
      return defaultExtractionRoot;
    }
    String home = System.getProperty("user.home");
    defaultExtractionRoot = Paths.get(home, ".wpilib", "nativecache").toString();
    return defaultExtractionRoot;
  }

  /**
   * Returns platform path.
   *
   * @return The current platform path.
   * @throws IllegalStateException Thrown if the operating system is unknown.
   */
  public static String getPlatformPath() {
    String filePath;
    String arch = System.getProperty("os.arch");

    boolean intel32 = "x86".equals(arch) || "i386".equals(arch);
    boolean intel64 = "amd64".equals(arch) || "x86_64".equals(arch);

    if (System.getProperty("os.name").startsWith("Windows")) {
      if (intel32) {
        filePath = "/windows/x86/";
      } else {
        filePath = "/windows/x86-64/";
      }
    } else if (System.getProperty("os.name").startsWith("Mac")) {
      filePath = "/osx/universal/";
    } else if (System.getProperty("os.name").startsWith("Linux")) {
      if (intel32) {
        filePath = "/linux/x86/";
      } else if (intel64) {
        filePath = "/linux/x86-64/";
      } else if (new File("/usr/local/frc/bin/frcRunRobot.sh").exists()) {
        filePath = "/linux/athena/";
      } else if ("arm".equals(arch) || "arm32".equals(arch)) {
        filePath = "/linux/arm32/";
      } else if ("aarch64".equals(arch) || "arm64".equals(arch)) {
        filePath = "/linux/arm64/";
      } else {
        filePath = "/linux/nativearm/";
      }
    } else {
      throw new IllegalStateException();
    }

    return filePath;
  }

  /**
   * Sets DLL directory.
   *
   * @param directory Directory.
   * @return DLL directory.
   */
  public static native String setDllDirectory(String directory);

  private static String setDllDirectoryShim(String directory) {
    if (getSkipSetDllDirectory()) {
      return null;
    } else {
      return setDllDirectory(directory);
    }
  }

  private static String getLoadErrorMessage(String libraryName, UnsatisfiedLinkError ule) {
    StringBuilder msg = new StringBuilder(512);
    msg.append(libraryName)
        .append(" could not be loaded from path\n" + "\tattempted to load for platform ")
        .append(getPlatformPath())
        .append("\nLast Load Error: \n")
        .append(ule.getMessage())
        .append('\n');
    if (System.getProperty("os.name").startsWith("Windows")) {
      msg.append(
          "A common cause of this error is missing the C++ runtime.\n"
              + "Download the latest at https://support.microsoft.com/en-us/help/2977003/the-latest-supported-visual-c-downloads\n");
    }
    return msg.toString();
  }

  /**
   * Extract a list of native libraries.
   *
   * @param <T> The class where the resources would be located
   * @param clazz The actual class object
   * @param resourceName The resource name on the classpath to use for file lookup
   * @return List of all libraries that were extracted
   * @throws IOException Thrown if resource not found or file could not be extracted
   */
  @RequiresApi(api = Build.VERSION_CODES.O)
  @SuppressWarnings("unchecked")
  public static <T> List<String> extractLibraries(Class<T> clazz, String resourceName)
      throws IOException {
    TypeReference<Map<String, Object>> typeRef = new TypeReference<Map<String, Object>>() {};
    ObjectMapper mapper = new ObjectMapper();
    Map<String, Object> map;
    try (java.io.InputStream stream = clazz.getResourceAsStream(resourceName)) {
      map = mapper.readValue(stream, typeRef);
    }

    java.nio.file.Path platformPath = Paths.get(getPlatformPath());
    String platform = platformPath.getName(0).toString();
    String arch = platformPath.getName(1).toString();

    Map<String, List<String>> platformMap = (Map<String, List<String>>) map.get(platform);

    List<String> fileList = platformMap.get(arch);

    String extractionPathString = getExtractionDirectory();

    if (extractionPathString == null) {
      String hash = (String) map.get("hash");

      String defaultExtractionRoot = getDefaultExtractionRoot();
      java.nio.file.Path extractionPath = Paths.get(defaultExtractionRoot, platform, arch, hash);
      extractionPathString = extractionPath.toString();

      setExtractionDirectory(extractionPathString);
    }

    List<String> extractedFiles = new ArrayList<>();

    byte[] buffer = new byte[0x10000]; // 64K copy buffer

    for (String file : fileList) {
      try (java.io.InputStream stream = clazz.getResourceAsStream(file)) {
        Objects.requireNonNull(stream);

        java.nio.file.Path outputFile = Paths.get(extractionPathString, new File(file).getName());
        extractedFiles.add(outputFile.toString());
        if (outputFile.toFile().exists()) {
          continue;
        }
        java.nio.file.Path parent = outputFile.getParent();
        if (parent == null) {
          throw new IOException("Output file has no parent");
        }
        parent.toFile().mkdirs();

        try (java.io.OutputStream os = Files.newOutputStream(outputFile)) {
          int readBytes;
          while ((readBytes = stream.read(buffer)) != -1) { // NOPMD
            os.write(buffer, 0, readBytes);
          }
        }
      }
    }

    return extractedFiles;
  }

  /**
   * Load a single library from a list of extracted files.
   *
   * @param libraryName The library name to load
   * @param extractedFiles The extracted files to search
   * @throws IOException If library was not found
   */
  public static void loadLibrary(String libraryName, List<String> extractedFiles)
      throws IOException {
    String currentPath = null;
    String oldDllDirectory = null;
    try {
      if (System.getProperty("os.name").startsWith("Windows")) {
        String extractionPathString = getExtractionDirectory();
        oldDllDirectory = setDllDirectoryShim(extractionPathString);
      }
      for (String extractedFile : extractedFiles) {
        if (extractedFile.contains(libraryName)) {
          // Load it
          currentPath = extractedFile;
          System.load(extractedFile);
          return;
        }
      }
      throw new IOException("Could not find library " + libraryName);
    } catch (UnsatisfiedLinkError ule) {
      throw new IOException(getLoadErrorMessage(currentPath, ule));
    } finally {
      if (oldDllDirectory != null) {
        setDllDirectoryShim(oldDllDirectory);
      }
    }
  }

  /**
   * Load a list of native libraries out of a single directory.
   *
   * @param <T> The class where the resources would be located
   * @param clazz The actual class object
   * @param librariesToLoad List of libraries to load
   * @throws IOException Throws an IOException if not found
   */
  public static <T> void loadLibraries(Class<T> clazz, String... librariesToLoad)
      throws IOException {
    // Extract everything

    @SuppressLint({"NewApi", "LocalSuppress"}) List<String> extractedFiles = extractLibraries(clazz, "/ResourceInformation.json");

    String currentPath = "";

    if (!getSkipSetDllDirectory()) {
      try {
        if (System.getProperty("os.name").startsWith("Windows")) {
          String extractionPathString = getExtractionDirectory();
          // Load windows, set dll directory
          currentPath = Paths.get(extractionPathString, "WindowsLoaderHelper.dll").toString();
          System.load(currentPath);
        }
      } catch (UnsatisfiedLinkError ule) {
        throw new IOException(getLoadErrorMessage(currentPath, ule));
      }
    }

    for (String library : librariesToLoad) {
      loadLibrary(library, extractedFiles);
    }
  }
}
