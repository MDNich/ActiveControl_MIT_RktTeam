import jpype
import os


# match your system as needed.
#os.environ['JAVA_HOME'] = '/Library/Java/JavaVirtualMachines/jdk1.8.0_202.jdk' 
os.environ['CLASSPATH'] = '../../../out/OpenRocket.jar'


jvm_path = jpype.getDefaultJVMPath()

jpype.startJVM(jvm_path)
print("SUCCESS")