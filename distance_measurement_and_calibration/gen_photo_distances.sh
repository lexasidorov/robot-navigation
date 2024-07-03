#!/bin/bash 

seq 170 30 499 | while read x
do
	file=$(ls out* | head -1)
	mv "$file" "$x".jpeg && echo rm "$file"
done