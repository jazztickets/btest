#!/bin/sh
grep btest -Ril . --exclude-dir=.git | xargs sed -i "s/btest/$1/g"
mv codelite/btest.project codelite/$1.project
mv codelite/btest.workspace codelite/$1.workspace
