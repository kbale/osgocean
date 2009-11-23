#Converts text files into UNIX format so that they
#can be used with osg2cpp

@echo off
FORFILES -M *.frag -C "cmd /c dos2unix @FILE" 
FORFILES -M *.vert -C "cmd /c dos2unix @FILE" 
