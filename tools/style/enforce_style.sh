#!/bin/bash
# Enforces the style on all .cpp and .h files in the src directory

ls build.gradle > /dev/null
if [ $? -ne 0 ]
then
    echo "Must be run from workspace directory"
    exit 1
fi

UnameOut=$(uname -s)
case "${UnameOut}" in
    Linux*)   FormatCmd="clang-format-5.0 -style=file";;
    Darwin*)  FormatCmd="clang-format -style=file";;
    *)        FormatCmd="clang-format -style-file"
esac

FormattableFiles=$(find src/main/cpp/src src/main/cpp/lib | egrep "\.(h|cpp)$" | paste -s -)
$FormatCmd -i $FormattableFiles
