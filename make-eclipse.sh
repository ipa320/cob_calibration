#!/bin/bash

for i in $( ls */manifest.xml | cut -d '/' -f 1 ); do
	echo "---------------------------------"
	echo " make eclipse project for $i"
	echo "---------------------------------"
	( cd $i && make eclipse-project )
done
