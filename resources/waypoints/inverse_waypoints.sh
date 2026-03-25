#!/bin/bash

INPUT="raceline.csv"
TMP="waypoints_tmp.csv"

tac "$INPUT" > "$TMP" && mv "$TMP" "$INPUT"

echo "Fichier inversé : $INPUT"