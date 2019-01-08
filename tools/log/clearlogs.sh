#!/bin/bash

# Clear all the logfiles on the rio

TEAM_NUMBER=$1
TARGET="lvuser@roborio-$TEAM_NUMBER-frc.local"

ssh -4 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $TARGET "rm log-*.txt"
