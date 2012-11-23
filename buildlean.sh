#!/bin/bash

[[ `diff arch/arm/configs/tuna_defconfig .config ` ]] && \
	{ echo "Unmatched defconfig!"; exit -1; } 

sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"-leanKernel-${1}\"/ .config

make ARCH=arm CROSS_COMPILE=/data/linaro/android-toolchain-eabi/bin/arm-linux-androideabi- -j2

cp arch/arm/boot/zImage mkboot/
#sed -i s/CONFIG_LOCALVERSION=\".*\"/CONFIG_LOCALVERSION=\"\"/ .config
cp .config arch/arm/configs/tuna_defconfig

cd mkboot
chmod 744 boot.img-ramdisk/sbin/lkflash
chmod 744 boot.img-ramdisk/sbin/checkv
chmod 744 boot.img-ramdisk/sbin/checkt
echo "making boot image"
./img.sh

zipfile="lk_gnex_jb42_v${1}.zip"
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
if [[ $1 != *dev* && $1 != *rc* ]]; then
	md5=`md5sum /tmp/boot.img | awk '{ print \$1 }'`
	cp /tmp/boot.img /tmp/boot-${1}.img
	url="http://imoseyon.host4droid.com/gnex/lkflash"
	if [[ $1 == *exp* ]]; then
	  if [[ $1 == *180* ]]; then
	    mf="latest180"
	  elif [[ $1 == *230* ]]; then
	    mf="latest230"
	  fi
	else 
	  mf="latest"
	fi
	echo "$url/boot-${1}.img $md5 ${1}" > /tmp/$mf
fi
if [[ $2 == "upload" ]]; then
  cd /lk/toro
  if [[ $1 != *dev* && $1 != *rc* ]]; then
	git log --pretty=format:"%aN: %s" -n 200 > /tmp/exp.log
  fi
  /data/utils/gnex_ftpupload42.sh $zipfile $1 $mf
fi
