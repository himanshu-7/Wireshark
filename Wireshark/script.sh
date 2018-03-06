#! /bin/bash

filename="campus_unique_ssid.txt"                                     # output data file 
interface="wlp6s0"                                           # monitor on this interface
duration="60"                                                #duration(in seconds) you want to capture data for. If you stop the script before the mentioned duration, data will not be saved



sudo airmon-ng start $interface
sudo tshark -i mon0 -a duration:$duration -I >> $filename														# start capturing 
letters=`cat $filename | grep SSID | grep -v "Malformed Packet" | awk -F "," '{print $6}' | awk -F "=" '{print $2}'| sort | uniq | wc -c`		 
lines=`cat $filename | grep SSID | grep -v "Malformed Packet" | awk -F "," '{print $6}' | awk -F "=" '{print $2}' | sort | uniq | wc -l` 		   		# count the total number of unique SSID in
letters=`expr $letters - $lines`
avg_length=`expr $letters / $lines`

cat $filename | grep SSID | grep -v "Malformed Packet" | awk -F "," '{print $6}' | awk -F "=" '{print $2}' |sort| uniq| tr -s [:space:] '\n'  >> word_frequency.txt		# create a file for word frequency


echo "Average Length of captured SSID's is: "$avg_length
echo "The top 10 most frequent words along with their frequency are dispayed below: "
for word in `cat word_frequency.txt`
do 
word_count=`grep $word word_frequency.txt | wc -l`
echo $word_count ":" $word >> top_counts.txt
done
cat top_counts.txt | sort -nr | head -10
echo "Top 10 most occuring SSID's are: "
cat $filename | grep SSID | grep -vi "malformed packet" | awk -F "," '{print $6}' | awk -F "=" '{print $2}' | sort | uniq -c | sort -nr | head 


rm -f word_frequency.txt
rm -f top_counts.txt
sudo airmon-ng stop mon0
sudo airmon-ng stop wlp6s0
