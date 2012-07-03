#!/sbin/bb/busybox ash
#
# lkconfig.sh - simple configuration manager for leankernel
#

bb="/sbin/bb/busybox"
left="minfreq maxfreq governor sriva srmpu srcore vminiva vminmpu vmincore srhigh tempcontrol gpuoc"
files="/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq \
	/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq \
	/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor \
	/sys/kernel/debug/smartreflex/sr_iva/autocomp \
	/sys/kernel/debug/smartreflex/sr_mpu/autocomp \
	/sys/kernel/debug/smartreflex/sr_core/autocomp \
	/sys/kernel/debug/smartreflex/sr_iva/vmin \
	/sys/kernel/debug/smartreflex/sr_mpu/vmin \
	/sys/kernel/debug/smartreflex/sr_core/vmin \
	/sys/kernel/debug/smartreflex/sr_mpu/enable_highvolt \
	/sys/class/misc/tempcontrol/templimit \
	/sys/devices/system/cpu/cpu0/cpufreq/gpu_oc"

userfile="/data/local/lk.conf"
configfile=$userfile

spitvalue()
{
  key=$2
  [[ $1 == "values" ]] && store=$cright || store=$files
  j=0
  k=0
  for x in $left; do
	j=$((j+1))
	[[ $key == $x ]] && break
  done
  for y in $store; do
	k=$((k+1)) 
	if [[ $j -eq $k ]]; then 
	  echo $y
	  break
	fi
  done
}
     
[ ! -f $userfile ] && configfile="/sbin/init.d/lk.conf"
temp=`$bb egrep ^[a-z] $configfile | $bb awk '{ print $1 }'`
cleft=`echo $temp | $bb tr -d '\r'`
cright=`$bb egrep ^[a-z] $configfile | $bb awk '{ print $2 }'`

[[ "$left" != "$cleft" ]] && exit

for i in $left; do
  echoval=`spitvalue values $i`
  sysfile=`spitvalue files $i`
  curval=`cat $sysfile`
  [[ "$curval" != "$echoval" ]] && echo $echoval > $sysfile
done


