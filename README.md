1.生成python的动态链接库
在scripts下面运行：

```
python3 setup.py build_ext --inplace
```

2.生成deb
在castlex_bringup下面运行：

```
bloom-generate rosdebian --os-name ubuntu --ros-distro melodic

fakeroot debian/rules binary
```

3.增加可执行权限
```bash
roscd castlex_voice_system/scripts/
sudo chmod +x *.py
```
