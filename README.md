

```markdown
 智能钻头控制仿真平台

本项目是一个基于 Python 的智能钻头控制仿真平台，通过结合 PID 控制和模糊逻辑控制，实现对电机-配流阀系统及液压执行器系统的仿真。用户可以通过图形化界面设置参数、选择仿真工况，并实时查看仿真结果。

 项目概览

本平台主要包含以下功能：
- 仿真模型：模拟电机、液压执行器系统以及其组合系统，并采用 PID 控制器与模糊控制器进行控制，实现对钻头姿态角、阀芯角度及液压推力的控制与调整。  
- 用户界面：使用 Tkinter 构建的图形化界面，可设置 PID 参数、选择仿真工况（初始偏差校正、扰动抑制、轨迹跟踪）、设定模型参数，并以多种图表样式展示仿真结果。  
- 数据可视化与导出：基于 Matplotlib 绘制动态图表，并支持将仿真图像导出为 PNG 文件、数据导出为 CSV 文件。

 项目功能与特点

- 直观的图形界面：通过友好的 UI 界面输入参数，选择不同的仿真场景，实时显示仿真进度与结果。  
- 多种控制策略：内置 PID 控制器和模糊控制器，提高系统响应性能。  
- 灵活的数据管理：支持生成多种图表（如线条图、散点图、填充图等），并能保存生成的图表和原始仿真数据。  
- 模块化设计：代码分为不同的文件，便于扩展和维护。

 项目结构

- ui.py (citeturn0file0)
  包含智能钻头控制仿真平台的主要 GUI 代码，实现了 `DrillSimUI` 类，其中集成了参数输入、仿真执行和图表绘制等功能。

- drillsimui.py (citeturn0file1)**  
  提供项目的主界面 `MainInterface`，通过该界面用户可选择进入不同的子模块（如智能钻头控制仿真平台）。

- main.py (citeturn0file2) 
  项目的入口文件，直接启动 `ui.py` 中的用户界面，适用于单一模块的直接仿真运行。

- models.py (citeturn0file3) 
  包含所有仿真所需的模型和控制器实现，包括电机-配流阀系统、液压执行器、组合系统、PID 控制器和模糊控制器，以及核心仿真函数 `run_simulation`。

 安装与依赖

 依赖项

本项目依赖以下 Python 库：
- Python 3.x
- Tkinter（Python 自带）
- NumPy
- Matplotlib
- scikit-fuzzy

你可以通过以下命令安装所需依赖：

```bash
pip install numpy matplotlib scikit-fuzzy
```

克隆代码库

使用 Git 克隆代码库：

```bash
git clone https://github.com/YourUsername/your-repo-name.git
cd your-repo-name
```

 使用说明

项目提供了两种启动方式，用户可以根据需求选择相应入口：

- 直接启动仿真界面  
  运行 `main.py` 文件，直接启动智能钻头控制仿真平台：

  ```bash
  python main.py
  ```

- 通过主界面选择模块  
  运行 `drillsimui.py` 文件，进入主界面后选择 “智能钻头控制仿真平台” 模块：

  ```bash
  python drillsimui.py
  ```

在仿真界面中，你可以按以下步骤操作：
1. 参数设置：输入 PID 控制参数以及（如需要）模型参数。
2. 选择仿真工况：选择初始偏差校正、扰动抑制或轨迹跟踪之一。
3. 选择图表样式：从多种图表样式（如线条图、散点图、填充图等）中选择一种显示方式。
4. 运行仿真：点击 “运行仿真” 按钮，启动仿真，并通过进度条显示计算进度。
5. 查看与导出结果：点击 “生成图表” 后可查看仿真图表，并可将图像和数据分别保存为 PNG 和 CSV 文件。

贡献

欢迎对本项目进行贡献和改进！如果你有任何建议或发现问题，请提交 Issue 或 Pull Request。

许可证

本项目采用 MIT 许可证，详情请参阅 LICENSE 文件。

致谢

本项目使用了 Tkinter 和 Matplotlib 实现图形化界面与数据可视化。
模糊逻辑控制部分基于 scikit-fuzzy 实现。

---

@成都理工大学严梁柱


```

