#!/bin/bash

VERSION=1.2.14

##
set -euo pipefail
set -x

url=https://github.com/ZinggJM/GxEPD2/archive/${VERSION}.tar.gz
name=GxEPD2-${VERSION}
tmpdir=$(mktemp -d)
wd=${PWD}
target=${wd}
copied_files_list=copied_files_gxepd2.list

cleanup(){
  cd ${wd}
  echo "Cleaning up ${tmpdir}"
  rm -R ${tmpdir}/*
  rmdir ${tmpdir}
}

trap cleanup EXIT

for f in $(cat ${copied_files_list}); do
  rm ${f} || true;
done
> ${copied_files_list}

mkdir -p ${target}
[[ -f ${name}.tar.gz ]] || curl -L $url > ${name}.tar.gz
cd ${tmpdir}
tar -xhvf ${wd}/${name}.tar.gz
cd ${name}/src

for f in GxEPD2.h epd/*.{cpp,h}; do
  [[ $f == epd/GxEPD2_1248.* ]] && continue # this uses SPI internally
  sed -i 's@#include "../GxEPD2_EPD.h@#include "GxEPD2_EPD.h@' ${f}
  if [[ $f != "GxEPD2.h" ]]; then
    sed -i '/^#include "GxEPD2.*/anamespace esphome {\nnamespace gxepd2_epaper {\n' ${f}
    echo -ne "}  // namespace gxepd2_epaper\n}  // namespace esphome\n" >> ${f}
    if [[ $f == *.cpp ]]; then 
      constructor_params=$(grep -h "GxEPD2_EPD(cs" ${f} | sed "s@cs, dc, rst, busy, @-1, -1, -1, -1, @" | head -n 1 )
      sed -i "s@^\([[:space:]]*GxEPD2_[0-9a-zA-Z_-]\+::GxEPD2_[0-9a-zA-Z_-]\+\)\((int8_t cs, int8_t dc, int8_t rst, int8_t busy) :\)\$@\1() :\n${constructor_params}\n{\n}\n\n\1\2@" $f
    elif [[ $f == *.h ]]; then
      sed -i "s@^\([[:space:]]*GxEPD2_[0-9a-zA-Z_-]\+\)\((int8_t cs, int8_t dc, int8_t rst, int8_t busy);\)\$@\1();\n\1\2@" $f
    fi
  else
    sed -i "/^#include <Arduino.h>$/d" ${f}
    sed -i "/^#include <SPI.h>$/d" ${f}
    sed -i "s@^\(class GxEPD2\)@namespace esphome {\nnamespace gxepd2_epaper {\n\1@" ${f}
    sed -i "s@^\(#endif\)@}  // namespace gxepd2_epaper\n}  // namespace esphome\n\1@" ${f}
  fi
  cp -a ${f} ${target}
  basename=${f##*/}
  echo ${basename} >> ${wd}/${copied_files_list}
done
