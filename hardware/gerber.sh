#!/bin/bash

BUILD_DIR=finalize
GERBER_DIR=gerber

zip_files() {
	SUBDIR_NAME=$1
	mkdir $GERBER_DIR/$SUBDIR_NAME
	cp $BUILD_DIR/$SUBDIR_NAME/*.{gbr,drl} $GERBER_DIR/$SUBDIR_NAME/
	pushd $GERBER_DIR/$SUBDIR_NAME
	zip $SUBDIR_NAME.zip *.{gbr,drl}
	popd
}

set -x
rm -rf $GERBER_DIR/*
zip_files board1
zip_files board2
