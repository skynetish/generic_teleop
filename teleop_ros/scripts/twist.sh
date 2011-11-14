#!/bin/bash

#------------------------------------------------------------------------------
# Usage
#   $1 - exit code
#------------------------------------------------------------------------------
usage() {
  echo ""
  echo "usage: ${0##*/} -t <topic> [options]"
  echo ""
  echo "  This program publishes a geometry_msgs/Twist message to the given topic."
  echo "  Unspecified velocities default to 0.  Options after '--' are passed through"
  echo "  to the 'rostopic pub' command."
  echo ""
  echo "  Required"
  echo "    -t --topic      target topic"
  echo ""
  echo "  Optional"
  echo "    -x              linear.x"
  echo "    -y              linear.y"
  echo "    -z              linear.z"
  echo "    --roll          angular.x"
  echo "    --pitch         angular.y"
  echo "    --yaw           angular.z"
  echo ""
  exit ${1}
}


#-----------------------------------------------------------
#Parse input parameters
#  $* parameters
#-----------------------------------------------------------
parse_input() {

  #Use getopt to parse arguments
  local getopt_output
  getopt_output=`getopt \
    --options     x:y:z:t: \
    --longoptions roll:,pitch:,yaw:,topic: \
    -n ${0##*/} \
    -- "$@"`

  #Check result of getopt
  [ $? -ne 0 ] && { echo "unable to parse command line"; exit 1; }

  #Set positional parameters to output of getopt
  eval set -- "$getopt_output"

  #Set options
  while true ; do
    case "$1" in

      #These are the parameters we allow
      -t|--topic)        TOPIC=$2;                        shift 2 ;;
      -x)                X=$2;                            shift 2 ;;
      -y)                Y=$2;                            shift 2 ;;
      -z)                Z=$2;                            shift 2 ;;
      --roll)            ROLL=$2;                         shift 2 ;;
      --pitch)           PITCH=$2;                        shift 2 ;;
      --yaw)             YAW=$2;                          shift 2 ;;

      #Last parameter recognised by getopt
      --) shift 1; break;;

      #Getopt shouldn't return anything else
      *) echo "unknown option ($1)" ; exit 1 ;;

    esac
  done

  #Extra parameters
  EXTRA=$*

  #Set defaults
  X="${X:-0}"
  Y="${Y:-0}"
  Z="${Z:-0}"
  ROLL="${ROLL:-0}"
  PITCH="${PITCH:-0}"
  YAW="${YAW:-0}"

  # Sanity checks
  [ "${TOPIC:-none}" == "none" ] && { echo "Invalid TOPIC ($TOPIC)";      usage 1; }
  [ ! -z "${X//[\-0-9.]}" ]      && { echo "Invalid velocity ($X)";       usage 1; }
  [ ! -z "${Y//[\-0-9.]}" ]      && { echo "Invalid velocity ($Y)";       usage 1; }
  [ ! -z "${Z//[\-0-9.]}" ]      && { echo "Invalid velocity ($Z)";       usage 1; }
  [ ! -z "${ROLL//[\-0-9.]}" ]   && { echo "Invalid velocity ($ROLL)";    usage 1; }
  [ ! -z "${PITCH//[\-0-9.]}" ]  && { echo "Invalid velocity ($PITCH)";   usage 1; }
  [ ! -z "${YAW//[\-0-9.]}" ]    && { echo "Invalid velocity ($YAW)";     usage 1; }
}

#-----------------------------------------------------------
#Main
#-----------------------------------------------------------
parse_input $*
rostopic pub ${EXTRA} ${TOPIC} geometry_msgs/Twist \
  "{ linear: { x: ${X} , y: ${Y} , z: ${Z} } , angular: { x: ${ROLL} , y: ${PITCH} , z: ${YAW} } }"
