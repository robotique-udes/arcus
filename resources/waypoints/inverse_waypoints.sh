#!/bin/bash

INPUT="waypoints.csv"
TMP="waypoints_tmp.csv"

tac "$INPUT" > "$TMP" && mv "$TMP" "$INPUT"

echo "Fichier inversé : $INPUT"