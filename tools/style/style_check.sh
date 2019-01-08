#!/bin/bash

exec < /dev/tty

UnameOut=$(uname -s)
case "${UnameOut}" in
    Linux*)   FormatCmd="clang-format-5.0 -style=file";;
    Darwin*)  FormatCmd="clang-format -style=file";;
    *)        FormatCmd="clang-format -style-file"
esac

Staged=$(git diff --cached --name-only | paste -s -)
StagedFormattable=$(git diff --cached --name-only --diff-filter=ACMRT lib src | egrep "\.(h|cpp)$" | paste -s -)


if [ -z "$StagedFormattable" ]
then
    echo "None of the files staged for commit need formatting"
    exit 0
else
    echo "The following formattable files are staged for commit: $StagedFormattable"
fi

diff -u <(cat $StagedFormattable) <($FormatCmd $StagedFormattable) > /dev/null

if [ $? -ne 0 ]
then
    while [ 1 ]
    do
        echo ""
        echo "There are style errors in files that you've touched."
        echo " - Cancel?  Type q (or press control-C any time)"
        echo " - View changes clang-format would like to make?  Type d"
        echo " - Automatically fix the errors for me? Press a"
        echo " - Force commit (ignore formatting error)? Type f"
        echo "For more info about style, check out docs/StyleGuide.md"
        printf "Enter Choice: "
        read -n1 UserResponse
        echo ""

        if [ "$UserResponse" == "q" ]
        then
            echo "Canceling"
            exit 1
        fi

        if [ "$UserResponse" == "d" ]
        then
            diff -u <(cat $StagedFormattable) <($FormatCmd $StagedFormattable) | less
        fi

        if [ "$UserResponse" == "a" ]
        then
            $FormatCmd -i $StagedFormattable
            git add $StagedFormattable
            echo "Files edited inline and re-added.  Proceeding with commit."
            exit 0
        fi

        if [ "$UserResponse" == "f" ]
        then
            echo "Proceeding with commit."
            exit 0
        fi

        if [[ "$UserResponse" != "c" && "$UserResponse" != "d" && "$UserResponse" != "a" && "$UserResponse" != "f" ]]
        then
            echo "Unrecognized option: $UserResponse"
        fi
    done
else
    echo "Files match suggested style.  Proceeding with commit."
fi
