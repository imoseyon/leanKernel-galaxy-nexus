#!/bin/sh

sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"-imoseyon-${1}\"/ .config

make -j2

cp arch/arm/boot/zImage mkboot/
sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"\"/ .config
cp .config arch/arm/configs/tuna_defconfig

cd mkboot
echo "making boot image"
./img.sh

zipfile="imoseyon_leanKernel_v${1}gnexus.zip"
if [ ! $4 ]; then
	echo "making zip file"
	cp boot.img ../zip
	cp boot.img /tmp
	cd ../zip
	rm -f *.zip
	zip -r $zipfile *
	rm -f /tmp/*.zip
	cp *.zip /tmp
fi
md5=`md5sum /tmp/boot.img | awk '{ print \$1 }'`
cp /tmp/boot.img /tmp/boot-${1}.img
echo "http://imoseyon.host4droid.com/boot-${1}.img $md5 ${1}" > /tmp/latest$1
