#!/bin/bash

counter ()
{
for (( i = 0; i < 10; i++ )); do
    sleep 1s
    echo "$i"
done
}

for (( j = 0; j < 10; j++ )); do
    counter
    xvkbd -text "\r" -window "rgbdslam" &> /dev/null
    echo "picture taken"
done

