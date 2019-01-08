# GreyDash

Forked from [FRC Dashboard](https://github.com/FRCDashboard/FRCDashboard), GreyDash is a custom HTML/CSS/JS based dashboard for interacting with the robot.

![](greydash.gif)

## Usage

This section will go over the usage of GreyDash.

### Installing dependencies

1. Follow the [installation guide for pynetworktables2js](https://pynetworktables2js.readthedocs.io/en/stable/#installation).
2. Familiarize yourself with the [usage of pynetworktables2js](https://pynetworktables2js.readthedocs.io/en/stable/#usage).
3. Follow instructions for starting the dashboard below.

### Starting

To start, first connect to the robot and open a driver station. Then, run `python3 -m pynetworktables2js --team=TEAM` where TEAM is the team number of the robot. You can also run `npm start` but you will need to change the team in the package.json, so this method is not reccomended.

### Linting
Please lint your JavaScript files--the easiest way is to run `npm lint`. This will fix most problems with your code, or let you know of ones it can't fix itself. You may ignore some undefined variables.

### Adding NetworkTables items

View [the dashboard guide](https://github.com/Team973/greybots-skeleton/blob/master/docs/DashboardGuide.md) in the docs folder.
