#!/bin/sh

sed -i s/CONFIG_LOCALVERSION=\"-imoseyon-.*\"/CONFIG_LOCALVERSION=\"-imoseyon-${1}\"/ .config

make -j2

cp .config arch/arm/configs/tuna_defconfig

cd mkboot
echo "making boot image"
./img.sh

if [ ! $4 ]; then
	echo "making zip file"
	cp boot.img ../zip
	cd ../zip
	rm *.zip
	zip -r $zipfile *
	rm /tmp/*.zip
	cp *.zip /tmp
fi
