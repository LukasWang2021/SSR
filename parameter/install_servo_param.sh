#!/bin/bash

usage() {
    echo -e "usage:"
    echo -e "\t""$(basename $0) 400w"
    echo -e "\t""or"
    echo -e "\t""$(basename $0) 600w"
    echo -e "note:"
    echo -e "\t""400w: install parameters of 400w which is in the directory of \"p7a_400w\""
    echo -e "\t""600w: install parameters of 600w which is in the directory of \"p7a_600w\""
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

if [ ! -d /root/install/share/configuration/machine/ ]
then
    mkdir -p /root/install/share/configuration/machine
fi

cp -a ${dir_}/${file_} /root/install/share/configuration/machine
