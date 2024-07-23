# Htcbot 远程图像配置手册

```shell
#  -i https://mirrors.aliyun.com/pypi/simple
#  -i https://pypi.tuna.tsinghua.edu.cn/simple
sudo apt update
pip3 install --upgrade pip setuptools wheel
pip3 install websockets -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install numpy
pip3 install opencv-python --prefer-binary -i https://pypi.tuna.tsinghua.edu.cn/simple
# PS: error tip need scikit-build
pip3 install scikit-build
```
