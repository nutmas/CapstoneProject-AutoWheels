import subprocess

commands = '''
cd darknet
make
./darknet detect cfg/yolov3.cfg yolov3.weights image_color/*.png
find . -name '*.jpg' | head -n 100 | xargs -I {} mv {} image_out/
'''

process = subprocess.Popen('/bin/bash', stdin=subprocess.PIPE, stdout=subprocess.PIPE)
out, err = process.communicate(commands.encode('utf-8'))
print(out.decode('utf-8'))



