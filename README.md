# polarbear_2025_algorithm_tutorial

深圳北理莫斯科大学 北极熊战队 2025赛季 算法组教程

## Dependency

- Ubuntu 22.04
- ROS Humble
- Clang, Clangd, Clang-format
- Black

推荐使用 VS Code IDE 进行开发。可一键安装推荐插件。

1. 官网安装并打开 VS Code。

2. 快捷键 `Ctrl+Shift+P` 打开命令面板。

3. 输入并选择 `Extensions: Install Workspace Recommended Extensions`。

这样，VS Code 会自动安装 [extensions.json](.vscode/extensions.json) 文件中列出的所有推荐插件。

## 代码规范

C/C++ 使用 clang-format 和 clang-tidy 自动进行代码格式化和代码检查。请将仓库根目录的 .clang-format 和 .clang-tidy 拷贝到你的工作空间。

Python 使用 black 进行代码格式化。请将仓库根目录的 .pylintrc 拷贝到你的工作空间。

[setting.json](.vscode/.templates/settings.json) 中 `"editor.formatOnSave": true` 已设置为 True，即每次保存文件时会自动进行代码格式化。

## 仓库结构

- .vscode

    提供了 ROS2 with VSCode 的模版配置文件，可以将 .vscode 中的文件拷贝到你的工作空间。详情可查看 <https://github.com/LihanChen2004/ros2_with_vscode>

- hwx_xxx_ws

    每次作业的工作空间，包含了每次作业的代码和文档。
