#!/bin/sh

#This script replaces all the paths to the css in the javadocs with the urls to the css. Original paths 
#in the directory don't work because the css files are removed during publishing.

startdir="$(pwd)/website/static/javadocs"
searchterm="href=.\+stylesheet.css"
replaceterm="href=\"https://docs.oracle.com/javase/8/docs/api/stylesheet.css"
searchterm2="href=.\+jquery-ui.css"
replaceterm2="href=\"http://code.jquery.com/ui/1.12.1/themes/base/jquery-ui.css"
for file in $(grep -l -R $searchterm $startdir)
	do
	sed -i "s|${searchterm}|${replaceterm}|g" $file
	sed -i "s|${searchterm2}|${replaceterm2}|g" $file
	echo "Modified: " $file
done

echo "Finished replacements."

#Removes all the previously existing css files
find $startdir -maxdepth 2 -name "*.css" -type f -delete 


