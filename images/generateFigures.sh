#! /bin/bash 

echo "Start to generate figures for the simulation"

python3 figure4a.py
python3 figure4b.py
python3 figure5.py
python3 figure7a.py
python3 figure7b.py
python3 figure8a.py
python3 figure8b.py
python3 figure9a.py
python3 figure9b.py
python3 figure10a.py
python3 figure10b.py
python3 figure11a.py
python3 figure11b.py
python3 figure11c.py
python3 figure11d.py
python3 figure13.py

../blender-3.3.1-linux-x64/blender --background --python ./figure12a.py 19
../blender-3.3.1-linux-x64/blender --background --python ./figure12b.py 19
../blender-3.3.1-linux-x64/blender --background --python ./figure12c.py 19
../blender-3.3.1-linux-x64/blender --background --python ./figure12d.py 19

echo "Finished generating figures"