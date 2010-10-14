import os
import time
import sys

###############################################

def readCopyright(root):
    copyrightFile = open(root+"/scripts/copyright_notice.txt","r")
    copyright = copyrightFile.readlines()
    copyrightFile.close()
    return copyright

###############################################

def shaderVarName( shaderName ):
    varName = shaderName.replace(".","_")
    return varName

###############################################

def isCurrent( shader, header ):
    if os.path.isfile( header ):
        shaderMod = os.path.getmtime(shader)
        headerMod = os.path.getmtime(header)
        if shaderMod < headerMod:
            return True
    return False

###############################################

def createInlShader( shaderFile, shaderVar, headerFile ):  
    file = open(shaderFile,'r')
    lines = file.readlines()
    file.close()
   
    oFile = open( headerFile, "w")
    
    for line in copyright:
        oFile.write(line)

    oFile.write("\nstatic const char " + shaderVar +"[] =")

    for line in lines:
        newLine = line.replace("\n","")
        oFile.write('\n\t"'+newLine+'\\n"')

    oFile.write(";\n");
    oFile.flush()
    oFile.close()

##############################################

srcRoot = ""

if len(sys.argv) < 2:
    print("Please specify the path to the root of the project")
    exit()
        
srcRoot = sys.argv[1]

shaderPath = srcRoot+"/resources/shaders/"
headerPath = srcRoot+"/include/osgOcean/shaders/"

shaderList = os.listdir( shaderPath )

skipped = 0
created = 0

print("--------------------------------\n")
print("\nProcessing shader files")
print("--------------------------------\n")

copyright = readCopyright(srcRoot)

for shader in shaderList:
    if shader.find("osgOcean_") > -1:
        if shader.rfind(".vert") > -1 or shader.rfind(".frag") > -1:
            sVar = shaderVarName(shader)
            hName = sVar + ".inl"
            hFile = headerPath + hName
            sFile = shaderPath + shader

            if isCurrent(sFile,hFile) == True:
                skipped += 1
                print("[skipped] " + sVar)
            else:
                createInlShader( sFile, sVar, hFile )
                created += 1
                print("[CREATED] " + sVar )
                
print("\n--------------------------------")
print(str(created)+"\tCreated")
print(str(skipped)+"\tUp to date")
print(str(skipped+created)+"\tTotal\n")
print("\n--------------------------------")    



