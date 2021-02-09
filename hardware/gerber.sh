#!/bin/bash

VERSION="v0.3"

zip_files() {
	SRC_DIR=$1
	DST_DIR=$2
	SUBDIR_NAME=$(basename $DST_DIR)
	mkdir $DST_DIR
	cp $SRC_DIR/*.{gbr,drl} $DST_DIR/
	pushd $DST_DIR
	zip JNTUB_${VERSION}_${SUBDIR_NAME}.zip *.{gbr,drl}
	popd
}

set -x
rm -rf gerber/*
zip_files finalize/board1 gerber/board1
zip_files finalize/board2 gerber/board2
zip_files front-panel/gerber gerber/panel
zip_files smt/ gerber/smt
