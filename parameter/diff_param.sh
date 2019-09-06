#!/bin/bash
working_dir=$(pwd)

ret_file=diff-servo-$(date +%y%m%d%H%M%S).txt

cur_param_dir=/root/install/share
cur_servo_param_file=/root/install/share/configuration/machine/servo_param.yaml

param_400=p7a_400w
param_600=p7a_600w

param_400_600_dir=""
param_400_600_file_list=""
cur_param_file=""
parram_400_600_file=""

opt=${1}

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

whichDestDir() {
    case ${opt} in
    400w)
        param_400_600_dir=${working_dir}/${param_400}
        ;;
    600w)
        param_400_600_dir=${working_dir}/${param_600}
        ;;
    *)
        echo -e "input wrong param"
        usage
        exit 2
        ;;
esac
}

findCurrentParamFile() {
    local filename=$(basename ${parram_400_600_file})
    if [ ${filename}="servo_param.yaml" ]
    then
        cur_param_file=${cur_servo_param_file}
    else
        cur_param_file=${cur_param_dir}${parram_400_600_file#*${param_400_600_dir}}
    fi
}

if [ ! $# -eq 1 ]
then
    echo "There is not a parameter."
    usage
    exit 1
fi

whichDestDir
if [ ! -d ${whichDestDir} ]
then
    echo -e "There is no director ${whichDestDir}"
    exit 3
fi

param_400_600_file_list=$(find ${param_400_600_dir} -name *.yaml)

touch ${ret_file}
for parram_400_600_file in ${param_400_600_file_list}
do
    findCurrentParamFile
    if [ ! -f ${cur_param_file} ]
    then
        echo -e "There is no $(basename ${cur_param_file}) in current parameter files."
        continue
    fi
    echo "<" ${cur_param_file} >> ${ret_file}
    echo ">" ${parram_400_600_file} >> ${ret_file}
    diff ${cur_param_file} ${parram_400_600_file} >> ${ret_file}
done

echo -e "diff is finished."
echo -e "Please check the result in ${ret_file}"

exit 0