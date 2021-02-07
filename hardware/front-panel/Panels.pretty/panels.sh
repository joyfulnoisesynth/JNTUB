#!/bin/bash

set -x

SVGS_DIR="/Users/Ben/Downloads/V1"

ARGS="-f 1.333333333333333333333333333333333 -c -p 0.4 -x -i "

run() {
	case $1 in
		chrd)
			svg2mod $ARGS $SVGS_DIR/chrd_paths/chrd.svg
			;;
		env)
			svg2mod $ARGS $SVGS_DIR/env_paths/env.svg
			;;
		env_alt)
			svg2mod $ARGS $SVGS_DIR/env_paths/env_alt.svg
			;;
		lfg)
			svg2mod $ARGS $SVGS_DIR/lfg_paths/lfg.svg
			;;
		lfg_alt)
			svg2mod $ARGS $SVGS_DIR/lfg_paths/lfg_alt.svg
			;;
		lfo)
			svg2mod $ARGS $SVGS_DIR/lfo_paths/lfo.svg
			;;
		lfo_alt)
			svg2mod $ARGS $SVGS_DIR/lfo_paths/lfo_alt.svg
			;;
		strang)
			svg2mod $ARGS $SVGS_DIR/strang_paths/strang.svg
			;;
		strang_alt)
			svg2mod $ARGS $SVGS_DIR/strang_paths/strang_alt.svg
			;;
		vcdo)
			svg2mod $ARGS $SVGS_DIR/vcdo_paths/vcdo.svg
			;;
		plain)
			svg2mod $ARGS $SVGS_DIR/plain.svg
			;;
	esac
}

if [[ $# -ge 1 ]]; then
	PANELS="$*"
else
	PANELS="chrd env env_alt lfg lfg_alt lfo lfo_alt strang strang_alt vcdo plain"
fi

for panel in $PANELS; do
	run $panel
done
