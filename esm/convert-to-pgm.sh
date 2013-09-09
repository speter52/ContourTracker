#!/usr/bin/env sh
# convertopgm.sh
# Martin Miller
# Created: 2013/09/09
# Converts each file in a dir to pgm and deletes the original.
# Dir should only contain images to be converted otherwise bad things will
# happen.
for i in $1/*
do
    PGM=${i%%.*}.pgm
    convert $i $PGM
    rm $i
done


