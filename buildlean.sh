#!/bin/bash

sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"-imoseyon-${1}\"/ .config

make -j2

cp arch/arm/boot/zImage mkboot/
sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"\"/ .config
cp .config arch/arm/configs/tuna_defconfig

cd mkboot
chmod 744 boot.img-ramdisk/sbin/lkflash
chmod 744 boot.img-ramdisk/sbin/checkv
chmod 744 boot.img-ramdisk/sbin/checkt
echo "making boot image"
./img.sh

zipfile="imoseyon_leanKernel_v${1}gnexus.zip"
if [ ! $4 ]; then
	rm -f /tmp/*.img
	echo "making zip file"
	cp boot.img ../zip
	cp boot.img /tmp
	cd ../zip
	rm -f *.zip
	zip -r $zipfile *
	rm -f /tmp/*.zip
	cp *.zip /tmp
fi
[[ $1 == *dev* ]] && exit
[[ $1 == *rc* ]] && exit
md5=`md5sum /tmp/boot.img | awk '{ print \$1 }'`
cp /tmp/boot.img /tmp/boot-${1}.img
if [[ $1 == *exp* ]]; then
  if [[ $1 == *180* ]]; then
    mf="latest180"
  elif [[ $1 == *230* ]]; then
    mf="latest230"
  else
    mf="latestnotrim"
  fi
else 
  mf="latest"
fi
echo "http://imoseyon.host4droid.com/boot-${1}.img $md5 ${1}" > /tmp/$mf
