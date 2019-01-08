# 2019-inseason
## FRC Team 973: The Greybots 2019 Code for Destination: Deep Space

[![Build Status](https://travis-ci.com/Team973/2019-inseason.svg?token=9qRQZ4Pb162wPMEfroVb&branch=master)](https://travis-ci.com/Team973/2019-inseason)
[![FRC Year](https://img.shields.io/badge/frc-2019-brightgreen.svg)](https://www.firstinspires.com/robotics/frc/game-and-season)
[![Language Type](https://img.shields.io/badge/language-c++-brightgreen.svg)](https://wpilib.screenstepslive.com/s/4485/m/13810)

## Getting Started
We use Gradle to build here.  For a walk-through on installing the build system on your computer, check out `docs/InstallGuide.md`.

## Making your first Pull Request
We do a feature-branch workflow here.  You write your features in your own branches.  When you finish writing your feature and you've tested it on the robot, you can make a pull request to get those changes merged into `dev`. When `dev` has been battle tested, we merge `dev` into `master`.  In this way `dev` is kinda stable and `master` is super duper stable.  When it's midnight before an event and the robot is speaking in tongues and Gradle isn't building and we're all grumpy, we revert back to the last commit in `master` because we know that anything in the `master` branch is guaranteed to work.  `master` might not always have the latest and greatest features, but it will always have something that the drivers can work with.

Check out `docs/YourFirstPR.md` for a walk-through on making a pull request.

## Style guide
Style is important with programming.  It can be the difference between your PR getting accepted and your PR getting declined.  Check out `docs/StyleGuide.md` for examples of what good style looks like in source code.

Other teams may have different style guides.  We use this one.  It doesn't really matter what style guide we use as long as it's consistent across the whole codebase.
