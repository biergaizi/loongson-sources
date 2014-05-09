#!/bin/bash


release=$(make kernelrelease)
export INSTALL_PATH=./${release}/boot
export INSTALL_MOD_PATH=./${release}

mkdir -p ${release}/boot
make modules_install install

rm -r ${release}/boot/vmlinux-${release}
rm -r ${release}/lib/modules/${release}/build
rm -r ${release}/lib/modules/${release}/source
cp COPYING ${release}

tar -cJf linux-${release}.tar.xz ${release}
rm -rf ${release}

gpg -o linux-${release}.tar.xz.sig -ab linux-${release}.tar.xz
