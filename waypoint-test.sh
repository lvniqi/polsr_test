#!/bin/sh
#"0=POLSR;1=OLSR;2=AODV;3=DSDV;"
#PROTOCOL="0 1 2 3"
PROTOCOL="0 1 2 3"
SPEED="0 2 4 6 8 10 12 14 16 18 20 22 24 26 28 30 32 34 36 38 40 45 50 55 60"
SEED="1 2 3 4 5"
echo UAVNET TEST

echo TEST Delay of Different Protocol In Different Speed
for protocol in $PROTOCOL
do
  for speed in $SPEED
  do
  	for seed in $SEED
			do
			echo protocol $protocol, speed $speed, seed $seed
			{
				../../waf --run "waypoint-test --protocol=$protocol --speed=$speed --seed=$seed --test=0 --time=500 --mobility=true"
			}&
			
			{
				../../waf --run "waypoint-test --protocol=$protocol --speed=$speed --seed=$seed --test=0 --time=500 --mobility=false"
			}&
		done
	wait
  done
done

echo TEST Throughput of Different Protocol In Different Speed
for protocol in $PROTOCOL
do
  for speed in $SPEED
  do
  	for seed in $SEED
			do
			echo protocol $protocol, speed $speed, seed $seed
			{
				../../waf --run "waypoint-test --protocol=$protocol --speed=$speed --seed=$seed --test=1 --time=500 --mobility=true"
			}&
			
			{
				../../waf --run "waypoint-test --protocol=$protocol --speed=$speed --seed=$seed --test=1 --time=500 --mobility=false"
			}&
		done
	wait
  done
done

