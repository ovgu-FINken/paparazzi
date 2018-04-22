#!/bin/sh
# this script sets up the environment to
#   - build pylon applications (for samples only when pylon is not installed in the standard directory /opt/pylon5) or
#   - run pylon applications using GenTL Producers
#
# use dot sourcing when calling this script
# note that there is no exit in error cases, to not destroy the outer shell


if [ $# != 1 ] ; then
	echo "Usage:"
	echo "  source pylon-setup-env.sh <path to pylon install dir>"
	echo ""
	echo "     By sourcing this script, the current environment is modified to"
	echo "     be able to run and build pylon applications."
else
	# we can't deduce the pylon root from $0 or $BASH_SOURCE because it is not available on busybox
	NEW_PYLON_ROOT=`readlink -f "$1"`

	if [ ! -d "$NEW_PYLON_ROOT" ]; then
		echo "Error: The directory '$NEW_PYLON_ROOT' (PYLON_ROOT) does not exist" 1>&2
	else
		if [ -n "$PYLON_ROOT" -a "$PYLON_ROOT" != "$NEW_PYLON_ROOT" ]; then
			echo "Notice: PYLON_ROOT was already set. It got replaced with '$NEW_PYLON_ROOT'" 1>&2
		fi

		# To build pylon applications, PYLON_ROOT has to be set
		export PYLON_ROOT=$NEW_PYLON_ROOT

		# determine arch, since pylon is using
		# different directories for different archs.
		ARCH=`uname -m`
		case "$ARCH" in
			x86_64)
				pylonlibdir="$PYLON_ROOT/lib64"
				systembits=64
				;;
			arm*)
				pylonlibdir="$PYLON_ROOT/lib"
				systembits=32
				;;
			*)
				pylonlibdir="$PYLON_ROOT/lib"
				systembits=32
				;;
		esac

		# set default path for GenTL Producers
		gentlpath=$pylonlibdir/gentlproducer/gtl
		if [ -d "$gentlpath" ]; then
			if [ -z "$GENICAM_GENTL32_PATH" ] && [ "$systembits" = "32" ]; then
				export GENICAM_GENTL32_PATH="$gentlpath"
			fi
			if [ -z "$GENICAM_GENTL64_PATH" ] && [ "$systembits" = "64" ]; then
				export GENICAM_GENTL64_PATH="$gentlpath"
			fi
		fi
	fi
fi
