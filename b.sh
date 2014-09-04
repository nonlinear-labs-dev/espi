!#/bin/bash

if [[ -z ${CROSS_COMPILE} ]]; then
	echo You have to set \$CROSS_COMPILE in order to use this build script!
	exit -1
fi

if [[ -z ${KSRC} ]]; then
	echo You have to set \$KSRC to point to your kernel sourcetree, in order to usre this buildscript!
	exit -1
fi

echo "Building..."
echo "  \$KSRC=${KSRC}"
echo "  \$CROSS_COMPILE=${CROSS_COMPILE}"


make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} KVER=3.13.6 KSRC=${KSRC}
