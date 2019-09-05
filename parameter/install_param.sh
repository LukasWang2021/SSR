#!/bin/bash

input_opt=${1}

rbf_file=soc_system.rbf
bin_file=ITEM.axf.bin
runtime_file=runtime
servo_param_file=servo_param.yaml

root_dir=${pwd}
fat_dir=/mnt/servo
param_dir=/root/install/share

current_dir=${root_dir}

src=""
dest=""

whichDest() {
    local name=${1}
    case ${name} in
    ${rbf_file} | ${bin_file})
        dest=${fat_dir}
        ;;
    ${runtime_file})
        dest=${param_dir}
        if [ ! -d ${dest} ]
        then
            mkdir -p ${dest}
        fi
        ;;
    ${servo_param_file})
        dest=${param_dir}/configuration/machine
        if [ ! -d ${dest} ]
        then
            mkdir -p ${dest}
        fi
        ;;
    *)
        echo "Wrong file name. Have no idea how to do."
        continue
        ;;
    esac

}

updateFile() {
    cp -rv ${src} ${dest} 
}

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

case ${input_opt} in
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

if [ -d ${dir_} ]
then
    cd ${dir_}
    echo -e "enter "${dir_}
else
    echo -e "Can't find the directory \"${dir_}\" in the current path: "$(pwd)
    exit 3
fi

file_list=$(ls)
for src in ${file_list}
do
    whichDest ${src}
    updateFile
done

cd ${root_dir}
echo -e "leave "${dir_}
echo -e "The installation of ${input_opt} is finished."