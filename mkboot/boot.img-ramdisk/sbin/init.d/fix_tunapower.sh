#!/sbin/bb/busybox ash
bb="/sbin/bb/busybox"
md5=`$bb md5sum /system/lib/hw/power.tuna.so | $bb awk '{ print $1 }'`
[[ $md5 == "a70ff42a1f19c6bc965acbbdbe4bfb3c" ]] && exit

mount -o rw,remount /system
$bb cp /system/lib/hw/power.tuna.so /system/lib/hw/power.tuna.so.bak
$bb cp /sbin/init.d/power.tuna.so /system/lib/hw
$bb chmod 644 /system/ilb/hw/power.tuna.so
mount -o ro,remount /system
