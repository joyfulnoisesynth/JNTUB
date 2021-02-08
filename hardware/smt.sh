#!/bin/bash

OUTPUT_DIR=smt
BACKUP_DIR=backup/smt
STAGING_DIR=tmp

set -x
mkdir -p $OUTPUT_DIR

# Copy existing contents of output dir to backup dir
rm -rf $BACKUP_DIR
mkdir $BACKUP_DIR
cp -R $OUTPUT_DIR/* $BACKUP_DIR/

# Clear out output dir
rm -rf $OUTPUT_DIR/*

cp JNTUB.kicad_pcb $OUTPUT_DIR/JNTUB_smt.kicad_pcb
scripts/smd_convert.py $OUTPUT_DIR/JNTUB_smt.kicad_pcb
