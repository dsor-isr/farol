#!/bin/bash

#echo $@
gains_val=`echo $@ |  sed s/" "/,/g | cut -d "," -f1-`
#echo $gains_val

rosservice call /PFUpdateGains "gains: [$gains_val]"