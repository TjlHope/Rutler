#!/bin/sh

# Get shell type, though this isn't very good as other shells will probably
# screw up the rest of the script
ext="${SHELL##*/}"

# Get the script execution
case "${ext}" in
    'bash')
	script="${BASH_SOURCE[0]}";;
    'zsh')
	script="${0}";;
    *)
	echo "Do not understand the workings of your shell," 1>&2
	echo "run the commands in this script manually." 1>&2
	exit 1
	;;
esac

# Get the dir of this script
script_dir="$(readlink -f "$(dirname "${script}")")"
# Add individual local packages to package path
for d in ${script_dir}/*
do
    [ -d "${d}" ] &&
	local_stacks="${local_stacks}:${d}"	# add to stacks path
done

# Source ros setup script and get PATH settings
if [ -f "/opt/ros/electric/setup.${ext}" ]
then
    . "/opt/ros/electric/setup.${ext}"	# source setup script
    [ -d "/opt/ros/electric/stacks" ] && {
	stacks_path="/opt/ros/electric/stacks"
	for d in /opt/ros/electric/stacks/*
	do
	    [ -d "${d}" ] &&
		stacks="${stacks}:${d}"		# add to stacks path
	done
    }
else
    while [ "${path}" != '/' ]
    do
	# Set path to be up one directory, using the script dir as the base
	path="$(readlink -f "${path-${script_dir}}/../")"
	if [ -f "${path}/ros/electric/ros/setup.${ext}" ]
	then
	    . "${path}/ros/electric/ros/setup.${ext}"	# source setup script
	    [ -d "${path}/ros/electric/stacks" ] && {
		stacks_path="${path}/ros/electric/stacks"
		for d in ${path}/ros/electric/stacks/*
		do
		    [ -d "${d}" ] &&
			stacks="${stacks}:${d}"		# add to stacks path
		done
	    }
	elif [ -f "${path}/ros/setup.${ext}" ]
	then
	    . "${path}/ros/setup.${ext}"		# source setup script
	    [ -d "${path}/stacks" ] && {
		stacks_path="${path}/stacks"
		for d in ${stacks_path}/*
		do
		    [ -d "${d}" ] &&
			stacks="${stacks}:${d}"		# add to stacks path
		done
	    }
	else
	    # Haven't found it so go to next iteration
	    continue
	fi
	break
    done
fi

# new PATHS to add to the start of ROS env vars
ws="${script_dir}${stacks_path+:}${stacks_path}"
pp="${script_dir}${stacks_path+:}${stacks_path}${local_stacks}${stacks}"

export ROS_WORKSPACE="${ws}${ROS_WORKSPACE+:}${ROS_WORKSPACE#${ws}}"
export ROS_PACKAGE_PATH="${pp}${ROS_PACKAGE_PATH+:}${ROS_PACKAGE_PATH#${pp}}"

# conveniance stuff:
alias .ros=". ${ws%%:*}/setup.sh"
# for non ubuntu:
if [ ! -f "/etc/lsb-release" ] ||
    [ "$(sed -ne 's:^DISTRIB_ID=.\(.*\).$:\1:p' <'/etc/lsb-release')" = 'Gentoo' ]
then
    export pass_seds='s:^.*<<<\s\+\(\S\+\)\s\+\[PASS\].*:\1:p'
    alias rosmake='rosmake --mark-installed --no-rosdep'
fi

# Keep enviroment polution down
unset ext script script_dir local_stacks stacks path ws pp
