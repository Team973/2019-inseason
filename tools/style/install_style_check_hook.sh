#!/bin/bash
# Installs tools/style/style_check.sh as the pre-commit hook

ls WORKSPACE > /dev/null
if [ $? -ne 0 ]
then
    echo "Must be run from workspace directory"
    exit 1
fi

# for the ln command, TARGET path is relative to DESTINATION folder
ln -sf ../../tools/style/style_check.sh .git/hooks/pre-commit
chmod +x .git/hooks/pre-commit

echo "Successfully installed pre-commit hook"
