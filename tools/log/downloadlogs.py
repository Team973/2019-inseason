#!/usr/bin/env python3

"""
Copy ALL the logs from the roborio to this directory
For each logfile, check the contents and see if there's a match number.  If
there is a match number, copy the file into the labeled directory and change
the name of the file to the match number (makes portmortem log-checking much
easier).
"""

from csv import DictReader
from os import listdir, path, system
from sys import argv

# TODO Check for team number in environment
TEAM_NUMBER = argv[1]

# TODO determinea better destination location for logfiles
DESTINATION_DIR_PARENT = path.expanduser("~/frc2019/973_robot_logs")
DESTINATION_DIR_RAW = DESTINATION_DIR_PARENT + 'raw/'
DESTINATION_DIR_LABELED = DESTINATION_DIR_PARENT + 'labeled/'

system('mkdir {}'.format(DESTINATION_DIR_PARENT))
system('mkdir {}'.format(DESTINATION_DIR_RAW))
system('mkdir {}'.format(DESTINATION_DIR_LABELED))

system('scp -4 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p lvuser@roborio-{}-frc.local:/home/lvuser/log-*.txt {}'.format(
    TEAM_NUMBER, DESTINATION_DIR_RAW))

for filename in listdir(DESTINATION_DIR_RAW):
    try:
       matchLabel = None
       with open(DESTINATION_DIR_RAW + filename, 'r') as f:
           reader = DictReader(f)
           for row in reader:
               if row['Match Identifier']:
                   matchLabel = row['Match Identifier']

       if matchLabel:
           system('cp {} "{}"'.format(
               DESTINATION_DIR_RAW + filename,
               DESTINATION_DIR_LABELED + matchLabel + '.csv'))
    except Exception as e:
        print( "Could not categorize logfile %s because exception %s" % ( filename, str( e ) ) )

print("Files located in " + DESTINATION_DIR_PARENT)
