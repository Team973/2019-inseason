# 2019-inseason
## FRC Team 973: The Greybots 2019 Code for Destination: Deep Space
### 2019 Houston World Champions!
### 2019 Orange County and Sacramento Regional Autonomous Award Winners!

[![Build Status](https://travis-ci.com/Team973/2019-inseason.svg?token=9qRQZ4Pb162wPMEfroVb&branch=master)](https://travis-ci.com/Team973/2019-inseason)
[![FRC Year](https://img.shields.io/badge/frc-2019-brightgreen.svg)](https://www.firstinspires.org/robotics/frc/game-and-season)
[![Language Type](https://img.shields.io/badge/language-c++-brightgreen.svg)](https://wpilib.screenstepslive.com/s/4485/m/13810)

## Getting Started
Welcome to the 2019 Robot Code for the World Champ robot: Fireball! We use Gradle to build here.  For a walk-through on installing the build system on your computer, check out `guides/InstallGuide.md`. Then, run `./gradlew build` in the top-level directory.

## Main features
Some unique and interesting features to look for:

- Custom vision alignment control algorithm used in teleop and sandstorm by running three PID loops off the roboRIO based off Limelight x-offset, y-offset (for real time distance calculations), and target skew/rotation
- Automated L3, buddy-L3, and L2 climb sequences
- Limelight as a source for driver feedback
- State machine to handle automated motions for simultaneous elevator, cargo, and hatch intake
- Current low-pass filtering for cargo intake
- Utilizing motion magic control mode on Talon SRXs for elevator motions
- GreyDash - custom, real-time dashboard primarily used for tuning PID and reading miscellaneous robot data (based on FRCDashboard pre-electron)
- GreyCTRE/GreySparkMax Wrapper - used to set default values for all Talon SRXs/Victor SPXs and Spark MAXs
- Coding style checker at commit following our own style guide (found in the `tools` folder)

## Contributors
Thank you to everyone who helped with our code for this year! Special thanks to the following.

### Students
- Kyle D (Lead Programmer): [@KyleD973](https://github.com/KyleD973)
- Chris M (Veteran): [@Chrismac112](https://github.com/Chrismac112)
- Chris L (Veteran): [@Chris2fourlaw](https://github.com/Chris2fourlaw)
- Luis V (New!): [@Luis-Velasco](https://github.com/Luis-Velasco)
- Dylan F (New!): [@Dylan-Fitchmun](https://github.com/Dylan-Fitchmun)

### Mentors
- Allen B: [@rubikscube4](https://github.com/rubikscube4)
- Andrew N: [@yabberyabber](https://github.com/yabberyabber)
- Oliver C: [@ocurr](https://github.com/ocurr)
- John P: [@AddictArts](https://github.com/AddictArts)
