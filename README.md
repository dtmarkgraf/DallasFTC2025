## NOTICE

This repository contains the public FTC SDK for the DECODE (2025-2026) competition season.

## Welcome!
This GitHub repository contains the source code that is used to build an Android app to control a *FIRST* Tech Challenge competition robot.  To use this SDK, download/clone the entire project to your local computer.

## Requirements
To use this Android Studio project, you will need Android Studio Ladybug (2024.2) or later.

To program your robot in Blocks or OnBot Java, you do not need Android Studio.

## Getting Started
If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html), and then migrate to the [OnBot Java Tool](https://ftc-docs.firstinspires.org/programming_resources/onbot_java/OnBot-Java-Tutorial.html) or to [Android Studio](https://ftc-docs.firstinspires.org/programming_resources/android_studio_java/Android-Studio-Tutorial.html) afterwards.

## Downloading the Project
If you are an Android Studio programmer, there are several ways to download this repo.  Note that if you use the Blocks or OnBot Java Tool to program your robot, then you do not need to download this repository.

* If you are a git user, you can clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

* You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the [Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) page for this repository.

* The Releases page also contains prebuilt APKs.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FIRST Tech Challenge Community site:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.

# Release Information

## Version 11.0 (20250827-105138)

### Enhancements

* OnBotJava now has the concept of a project.  
  A project is a collection of related files.  A project may be chosen by selecting 'Example Project'
  from the 'File type:' dropdown.  Doing so will populate the dropdown to the immediate right with 
  a list of projects to choose from.
  When selecting a project all of the related files appear in the left pane of the workspace 
  underneath a directory with the chosen project name.
  This is useful for example for ConceptExternalHardwareClass which has a dependency upon
  RobotHardware.  This feature simplifies the usage of this Concept example by automatically
  pulling in dependent classes.
* Adds support for AndyMark ToF, IMU, and Color sensors.
* The Driver Station app indicates if WiFi is disabled on the device.
* Adds several features to the Color Processing software:
  * DECODE colors `ARTIFACT_GREEN` and `ARTIFACT_PURPLE`
  * Choice of the order of pre-processing steps Erode and Dilate
  * Best-fit preview shape called `circleFit`, an alternate to the existing `boxFit`
  * Sample OpMode `ConceptVisionColorLocator_Circle`, an alternate to the renamed `ConceptVisionColorLocator_Rectangle`
* The Driver Station app play button has a green background with a white play symbol if
  * the driver station and robot controller are connected and have the same team number
  * there is at least one gamepad attached
  * the timer is enabled (for an Autonomous OpMode)
* Updated AprilTag Library for DECODE. Notably, getCurrentGameTagLibrary() now returns DECODE tags.
  * Since the AprilTags on the Obelisk should not be used for localization, the ConceptAprilTagLocalization samples only use those tags without the name 'Obelisk' in them.
* OctoQuad I2C driver updated to support firmware v3.x 
  * Adds support for odometry localizer on MK2 hardware revision
  * Adds ability to track position for an absolute encoder across multiple rotations
  * Note that some driver APIs have changed; minor updates to user software may be required
  * Requires firmware v3.x. For instructions on updating firmware, see
    https://github.com/DigitalChickenLabs/OctoQuad/blob/master/documentation/OctoQuadDatasheet_Rev_3.0C.pdf
 * The hardware loop has been updated for better performance.
