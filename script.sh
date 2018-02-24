#! /bin/bash

sudo airmon-ng start wlp6s0
sudo tshark -i mon0 -a duration:1800 -I >> data_main.txt
letters=`cat data_main.txt | awk -F "," '{print $6}' | sort | uniq | awk -F "=" '{print $2}' | wc -c`
lines=`cat data_main.txt | awk -F "," '{print $6}' | sort | uniq | wc -l` 
echo $lines
echo $letters
avg_length=`expr $letters / $lines`
echo $avg_length
