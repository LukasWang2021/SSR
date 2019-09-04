#!/bin/bash

usage() {
    echo -e "usage:"
    echo -e "\t""$(basename $0) 400w"
    echo -e "\t""or"
    echo -e "\t""$(basename $0) 600w"
    echo -e "note:"
    echo -e "\t""400w: campare the current servo parameters with what is in the directory of \"p7a_400w\""
    echo -e "\t""600w: campare the current servo parameters with what is in the directory of \"p7a_600w\""
    echo -e
    echo -e "the result file is /root/diff-servo-<time>.txt"
    echo -e "In the result file, \"<\" means the contents of the current parameters, and \">\" means the contents of the parameter of 400w/600w."
    echo -e
}

if [ ! $# -eq 1 ]
then
    echo "There is not a parameter."
    usage
    exit 1
fi

file_=servo_param.yaml
case $1 in
400w)
    dir_=p7a_400w;;
600w)
    dir_=p7a_600w;;
*)
    echo -e "input wrong param"
    usage
    exit 2
    ;;
esac

if [ ! -d ${dir_} ]
then
    echo -e "Can't find the directory \"${dir_}\" in the current path: "$(pwd)
    exit 3
fi

if [ ! -f ${dir_}/${file_} ]
then
    echo -e "Can't find the file \"${file_}\" in the current path: ./"${dir_}
    exit 4
fi

current_param=/root/install/share/configuration/machine/${file_}
if [ ! -f ${current_param} ]
then
    echo -e "Can't find the current servo parameter file: ${current_param}"
fi

diff ${current_param} ${dir_}/${file_} > /root/diff-servo-$(date +%y%m%d%H%M%S).txt