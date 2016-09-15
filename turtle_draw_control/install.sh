#!/usr/bin/env bash

ANDROID_CORE_LOCATION_FILE=android_core.location

if [ -f $ANDROID_CORE_LOCATION_FILE ]; then
    ANDROID_CORE_PROJECT=$(cat $ANDROID_CORE_LOCATION_FILE)
else
    echo "Enter the location of the android_core project:"
    read ANDROID_CORE_PROJECT
    ANDROID_CORE_PROJECT="${ANDROID_CORE_PROJECT/#\~/$HOME}" # Replace ~ with $HOME

    echo "Storing configuration in $ANDROID_CORE_LOCATION_FILE"
    echo $ANDROID_CORE_PROJECT > $ANDROID_CORE_LOCATION_FILE
fi

TURTLE_DRAW_CONTROL_LOCATION=$(pwd)

cd $ANDROID_CORE_PROJECT
ln -s $TURTLE_DRAW_CONTROL_LOCATION turtle_draw_control
