#! /bin/sh

# Enable pylon logging for this run. The logfile will be created in /tmp/pylonLog.txt.
# You can use other pylon-based applications as a parameter to this script file to run it with logging enabled.
# If you just start this script with no args the pylonViewer will be started.

# determine the directory this script resides in
this_script_full_path=`readlink -f "$0"` # Absolute path this script
this_script_directory=`dirname "$this_script_full_path"` # the directory

# this script is in the $PYLON_ROOT/bin,
# so set PYLON_ROOT to the parent of this dir
# if you put this script into a different directory
# you'll need to modify the "${this_script_directory}/.." accordingly.

# Note: using readlink so the relative path will be converted to an absolute
PYLON_ROOT=`readlink -f "${this_script_directory}/.."`

# use the installed property file
PROPERTIES_INSTALLED="$PYLON_ROOT/share/pylon/log/config/DebugLoggingUnix.properties"
if [ -e "$PROPERTIES_INSTALLED" ]; then
    export GENICAM_LOG_CONFIG_V3_0="$PROPERTIES_INSTALLED"
fi

# when present use an editable copy in the $HOME directory
PROPERTIES_HOME="$HOME/DebugLoggingUnix.properties"
if [ -e "$PROPERTIES_HOME" ]; then
    export GENICAM_LOG_CONFIG_V3_0="$PROPERTIES_HOME"
fi

if [ -z "$GENICAM_LOG_CONFIG_V3_0" ]; then
    echo "Error: File DebugLoggingUnix.properties not found"
    exit 1
fi

# start the application
echo "Logging activated using $GENICAM_LOG_CONFIG_V3_0"
echo "Waiting for application to exit ..."
echo

app_to_start="$1"
if [ -z "$app_to_start" ]; then
    app_to_start="${this_script_directory}/PylonViewerApp"
fi

"$app_to_start"

if [ -e "/tmp/pylonLog.txt" ]; then
    echo
    echo "To view the logfile please open /tmp/pylonLog.txt"
fi
