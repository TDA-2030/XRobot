# sdk_standalone

# Windows 64位系统下 Python SDK 安装指南

1. 下载 [MinGW64](https://github.com/skeeto/w64devkit/releases/download/v1.18.0/w64devkit-1.18.0.zip) 编译工具链后解压，假设目录为 `D:/w64devkit`

2. cd 到目录 `V4-sdk-standalone/sdk_py` 并设置环境变量
``` shell
set PATH=D:/w64devkit/bin;%PATH%
```

3. 运行安装脚本 `./install.bat`

4. 添加语法提示（可选），拷贝 `motormaster.pyi` 到 python 安装包 `motormaster-0.0.1-py3.8-win-amd64.egg` 目录下