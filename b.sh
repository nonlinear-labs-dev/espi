#!/bin/bash

if [[ -z ${CROSS_COMPILE} ]]; then
	echo You have to set \$CROSS_COMPILE in order to use this build script!
	exit -1
fi

if [[ -z ${KSRC} ]]; then
	echo You have to set \$KSRC to point to your kernel sourcetree, in order to usre this buildscript!
	exit -1
fi


if [[ $1 == "clean" ]]; then
	echo "Cleaning..."
	make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} -C ${KSRC} M=$(pwd) clean
elif [[ $1 == "reload" ]]; then
	scp espi_driver.ko root@192.168.7.2:/lib/modules/3.18.4/extra/espi_driver.ko
	ssh -l root 192.168.7.2 "killall playground; rmmod espi_driver; modprobe espi_driver"
else
	echo "Building..."
	make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} -C ${KSRC} M=$(pwd)
fi

