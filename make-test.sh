#!/bin/bash

for i in $( ls */manifest.xml | cut -d '/' -f 1 ); do
	echo "---------------------------------"
	echo " make test for $i"
	echo "---------------------------------"
	( cd $i && make test )
done
