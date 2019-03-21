#!/bin/bash

upload_server=$1

projectdir=`git rev-parse --show-toplevel`
if [ -z "$projectdir" ]; then
	echo "No .git directory found"
	exit 1
fi

outputdir="$projectdir/deployment/out"
cd "$projectdir"
mkdir -p working
mkdir -p "$outputdir"

version=`grep 'GAME_VERSION=".*"' -o CMakeLists.txt | sed -r "s/GAME_VERSION=\"(.*)\"/\1/"`

build() {

	bits=$1

	# get mingw prefix
	if [ $bits -eq "32" ]; then
		arch=i686-w64-mingw32
	else
		arch=x86_64-w64-mingw32
	fi

	# run cmake
	builddir="$projectdir/build/mingw$bits"
	mkdir -p "$builddir"
	cd "$builddir"
	cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake/mingw${bits}.cmake ../../

	# build
	make -j`nproc`

	if [ $? -ne 0 ]; then
		echo "failed $builddir"
		exit
	fi

	cd "$projectdir"

	cp /usr/$arch/bin/{libstdc++-6.dll,libwinpthread-1.dll,libgcc_*.dll} working/

	gitver=`git log --oneline | wc -l`
	mv bin/Release/btest.exe working/
	cp README working/
	#chmod +x working/run_*.bat

	archive_base=btest-${version}r${gitver}-win${bits}
	archive=${archive_base}.zip
	ln -s working "$archive_base"
	zip -r "$archive" "$archive_base" -x /"${archive_base}"/btest
	rm "${archive_base}"

	rm working/btest.exe
	rm working/*.dll
	rm working/README

	if [ -n "$upload_server" ]; then
		scp $archive $upload_server:web/files/
	fi

	mv $archive "$outputdir"
}

if [ -n "$upload_server" ]; then
	ssh $upload_server rm -f web/files/btest*.zip
fi

rm -f "$outputdir"/btest*.zip

#build 32
build 64
