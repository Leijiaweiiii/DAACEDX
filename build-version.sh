#!/bin/bash
#
# File:   build-version.sh
# Author: Boris Sorochkin boris@nestlogic.com
#
# Created on 14-Nov-2019, 19:38:58
#
DATE=$(date -u +"%Y.%m.%d")
VER=$(find .. -name *${DATE}*hex|cut -f 6 -d . |sort -n |tail -1)
VER=$(($VER+01))
TAG=$(printf "%s.%02d" ${DATE} ${VER})
git tag -f "${TAG}"
make clean
make -j 4
cp -v dist/v2_10/production/DAACED.X.production.hex ../${TAG}.hex