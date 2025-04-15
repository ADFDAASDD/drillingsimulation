import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import csv  # 导入 CSV 模块

# 从 models.py 中导入运行仿真函数
from models import run_simulation

import matplotlib.pyplot as plt
from matplotlib import rcParams

# 设置 Matplotlib 使用支持中文的字体
rcParams['font.sans-serif'] = ['Microsoft YaHei']  # 使用微软雅黑
rcParams['axes.unicode_minus'] = False  # 正常显示负号
rcParams['grid.linestyle'] = '--'  # 设置网格线样式
rcParams['axes.facecolor'] = '#f5f5f5'  # 设置图表背景色

# 创建主窗口和UI组件
class DrillSimUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("智能钻头控制仿真平台")
        self.geometry("1920x1080")  # 设置窗口大小
        self.create_widgets()

    def create_widgets(self):
        # 参数设置区域
        param_frame = tk.LabelFrame(self, text="PID 控制参数", padx=10, pady=10)
        param_frame.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        tk.Label(param_frame, text="Kp（比例增益，Proportional Gain）θ:").grid(row=0, column=0, sticky="e")
        tk.Label(param_frame, text="Ki（积分增益，Integral Gain）θ:").grid(row=1, column=0, sticky="e")
        tk.Label(param_frame, text="Kd（微分增益，Derivative Gain）θ:").grid(row=2, column=0, sticky="e")
        self.kp_entry = tk.Entry(param_frame, width=8)
        self.ki_entry = tk.Entry(param_frame, width=8)
        self.kd_entry = tk.Entry(param_frame, width=8)
        self.kp_entry.grid(row=0, column=1)
        self.ki_entry.grid(row=1, column=1)
        self.kd_entry.grid(row=2, column=1)
        # 设置默认PID参数
        self.kp_entry.insert(0, "40")
        self.ki_entry.insert(0, "5")
        self.kd_entry.insert(0, "5")

        # 仿真工况选择区域
        scenario_frame = tk.LabelFrame(self, text="仿真工况", padx=10, pady=10)
        scenario_frame.grid(row=0, column=1, sticky="w", padx=10, pady=10)
        self.scenario_var = tk.IntVar(value=1)
        tk.Radiobutton(scenario_frame, text="初始偏差校正", variable=self.scenario_var, value=1).grid(row=0, column=0, sticky="w")
        tk.Radiobutton(scenario_frame, text="扰动抑制", variable=self.scenario_var, value=2).grid(row=1, column=0, sticky="w")
        tk.Radiobutton(scenario_frame, text="轨迹跟踪", variable=self.scenario_var, value=3).grid(row=2, column=0, sticky="w")

        # 绘图风格选择区域
        style_frame = tk.LabelFrame(self, text="图表绘制风格", padx=10, pady=10)
        style_frame.grid(row=1, column=0, sticky="w", padx=10, pady=10)
        self.style_var = tk.StringVar(value="曲线图")
        style_options = ["线条图", "散点图", "填充图", "堆叠图", "对比强烈线条"]
        style_menu = tk.OptionMenu(style_frame, self.style_var, *style_options)
        style_menu.grid(row=1, column=1, sticky="w")

        # 图形显示区域
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312, sharex=self.ax1)
        self.ax3 = self.fig.add_subplot(313, sharex=self.ax1)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().grid(row=2, column=0, columnspan=2, padx=10, pady=10)

        # 运行仿真和保存图像按钮
        run_button = tk.Button(self, text="运行仿真", command=self.on_run, bg="#4CAF50", fg="white", font=("Arial", 12))
        run_button.grid(row=1, column=1, padx=10, pady=10)
        generate_button = tk.Button(self, text="生成图表", command=self.on_generate, bg="#FF9800", fg="white", font=("Arial", 12))
        generate_button.grid(row=1, column=2, padx=10, pady=10)
        save_button = tk.Button(self, text="保存图像", command=self.on_save, bg="#2196F3", fg="white", font=("Arial", 12))
        save_button.grid(row=1, column=3, padx=10, pady=10)
        save_data_button = tk.Button(self, text="保存数据", command=self.on_save_data, bg="#FFC107", fg="white", font=("Arial", 12))
        save_data_button.grid(row=1, column=4, padx=10, pady=10)

        # 进度条
        self.progress_label = tk.Label(self, text="计算进度：")
        self.progress_label.grid(row=3, column=0, padx=10, pady=10, sticky="w")
        self.progress = ttk.Progressbar(self, orient="horizontal", length=300, mode="determinate")
        self.progress.grid(row=3, column=1, padx=10, pady=10, sticky="w")

    def on_run(self):
        # 运行仿真并存储计算结果
        scenario = self.scenario_var.get()
        try:
            Kp = float(self.kp_entry.get())
            Ki = float(self.ki_entry.get())
            Kd = float(self.kd_entry.get())
        except ValueError:
            print("请输入有效的数字作为PID参数")
            return

        # 更新进度条
        self.progress["value"] = 0
        self.update_idletasks()

        # 运行仿真并计算
        time_list, phi_list, theta_list, phi_des_list = run_simulation(scenario, Kp, Ki, Kd, update_progress=self.update_progress)

        # 存储仿真计算结果
        self.time_list = time_list
        self.phi_list = phi_list
        self.theta_list = theta_list
        self.phi_des_list = phi_des_list

        # 计算角度和液压推力
        self.phi_deg = np.degrees(self.phi_list)
        self.theta_deg = np.degrees(self.theta_list)
        K_stiff = 1000.0
        self.F_h = K_stiff * np.array(self.phi_list)

        # 提示计算完成
        messagebox.showinfo("计算完成", "仿真计算已完成！请选择图表风格后点击生成图表。")

    def update_progress(self, value):
        # 更新进度条
        self.progress["value"] = value
        self.update_idletasks()

    def on_generate(self):
        if not hasattr(self, 'time_list'):
            print("请先运行仿真！")
            return

        selected_style = self.style_var.get()

        # 清空图表
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        if selected_style == "线条图":
            self.ax1.plot(self.time_list, self.phi_deg, 'b-', label='姿态角', linewidth=2)
        elif selected_style == "散点图":
            self.ax1.scatter(self.time_list, self.phi_deg, color='blue', label='姿态角', s=30)
        elif selected_style == "填充图":
            self.ax1.fill_between(self.time_list, self.phi_deg, color='skyblue', alpha=0.5, label='姿态角')
        elif selected_style == "堆叠图":
            self.ax1.fill_between(self.time_list, self.phi_deg, color='lightgreen', label='姿态角')
        elif selected_style == "对比强烈线条":
            self.ax1.plot(self.time_list, self.phi_deg, 'k-', label='姿态角', linewidth=3)

        self.ax1.set_ylabel('姿态角 φ (°)', fontsize=12)
        self.ax1.set_title('智能钻头控制仿真', fontsize=14, fontweight='bold')

        self.ax2.plot(self.time_list, self.F_h, 'g-', label='液压推力', linewidth=2)
        self.ax2.set_ylabel('侧向推力 F_h (N)', fontsize=12)

        self.ax3.plot(self.time_list, self.theta_deg, 'm-', label='阀芯角度', linewidth=2)
        self.ax3.set_ylabel('阀芯角度 θ (°)', fontsize=12)
        self.ax3.set_xlabel('时间 t (s)', fontsize=12)

        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax3.grid(True)

        self.canvas.draw()

    def on_save(self):
        file_path = filedialog.asksaveasfilename(title="保存图像",
                                                 filetypes=[("PNG Image", "*.png")],
                                                 defaultextension=".png")
        if file_path:
            self.fig.savefig(file_path)
            print(f"图像已保存: {file_path}")

    def on_save_data(self):
        # 保存数据为CSV文件
        file_path = filedialog.asksaveasfilename(title="保存数据",
                                                 filetypes=[("CSV File", "*.csv")],
                                                 defaultextension=".csv")
        if file_path:
            # 保存数据到CSV文件
            with open(file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                # 写入标题行
                writer.writerow(['Time (s)', '姿态角 φ (°)', '阀芯角度 θ (°)', '侧向推力 F_h (N)'])
                # 写入每一行数据
                for t, phi, theta, F in zip(self.time_list, self.phi_deg, self.theta_deg, self.F_h):
                    writer.writerow([t, phi, theta, F])
            print(f"数据已保存: {file_path}")


if __name__ == "__main__":
    app = DrillSimUI()
    app.mainloop()




import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import csv

# 从 models.py 中导入仿真函数
from models import run_simulation

import matplotlib.pyplot as plt
from matplotlib import rcParams

# 设置 Matplotlib 使用中文字体和其他样式
rcParams['font.sans-serif'] = ['Microsoft YaHei']
rcParams['axes.unicode_minus'] = False
rcParams['grid.linestyle'] = '--'
rcParams['axes.facecolor'] = '#f5f5f5'

# 创建主窗口和UI组件
class DrillSimUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("智能钻头控制仿真平台")
        self.geometry("1920x1080")
        self.create_widgets()

    def create_widgets(self):
        # PID 控制参数区域
        param_frame = tk.LabelFrame(self, text="PID 控制参数", padx=10, pady=10)
        param_frame.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        tk.Label(param_frame, text="Kp (比例增益):").grid(row=0, column=0, sticky="e")
        tk.Label(param_frame, text="Ki (积分增益):").grid(row=1, column=0, sticky="e")
        tk.Label(param_frame, text="Kd (微分增益):").grid(row=2, column=0, sticky="e")
        self.kp_entry = tk.Entry(param_frame, width=8)
        self.ki_entry = tk.Entry(param_frame, width=8)
        self.kd_entry = tk.Entry(param_frame, width=8)
        self.kp_entry.grid(row=0, column=1)
        self.ki_entry.grid(row=1, column=1)
        self.kd_entry.grid(row=2, column=1)
        self.kp_entry.insert(0, "40")
        self.ki_entry.insert(0, "5")
        self.kd_entry.insert(0, "5")

        # 仿真工况选择区域
        scenario_frame = tk.LabelFrame(self, text="仿真工况", padx=10, pady=10)
        scenario_frame.grid(row=0, column=1, sticky="w", padx=10, pady=10)
        self.scenario_var = tk.IntVar(value=1)
        tk.Radiobutton(scenario_frame, text="初始偏差校正", variable=self.scenario_var, value=1).grid(row=0, column=0, sticky="w")
        tk.Radiobutton(scenario_frame, text="扰动抑制", variable=self.scenario_var, value=2).grid(row=1, column=0, sticky="w")
        tk.Radiobutton(scenario_frame, text="轨迹跟踪", variable=self.scenario_var, value=3).grid(row=2, column=0, sticky="w")

        # 模型参数设置区域
        model_frame = tk.LabelFrame(self, text="模型参数", padx=10, pady=10)
        model_frame.grid(row=0, column=2, sticky="w", padx=10, pady=10)
        tk.Label(model_frame, text="J (电机转动惯量):").grid(row=0, column=0, sticky="e")
        tk.Label(model_frame, text="B (阻尼系数):").grid(row=1, column=0, sticky="e")
        tk.Label(model_frame, text="Kt (电机力矩常数):").grid(row=2, column=0, sticky="e")
        tk.Label(model_frame, text="K_h (液压增益常数):").grid(row=3, column=0, sticky="e")
        tk.Label(model_frame, text="tau_h (时间常数):").grid(row=4, column=0, sticky="e")
        tk.Label(model_frame, text="spool_max (最大角):").grid(row=5, column=0, sticky="e")
        tk.Label(model_frame, text="t_end (仿真时间):").grid(row=6, column=0, sticky="e")
        tk.Label(model_frame, text="dt_sim (仿真步长):").grid(row=7, column=0, sticky="e")

        self.J_entry = tk.Entry(model_frame, width=8)
        self.B_entry = tk.Entry(model_frame, width=8)
        self.Kt_entry = tk.Entry(model_frame, width=8)
        self.Kh_entry = tk.Entry(model_frame, width=8)
        self.tauh_entry = tk.Entry(model_frame, width=8)
        self.spool_entry = tk.Entry(model_frame, width=8)
        self.t_end_entry = tk.Entry(model_frame, width=8)
        self.dt_sim_entry = tk.Entry(model_frame, width=8)

        self.J_entry.grid(row=0, column=1)
        self.B_entry.grid(row=1, column=1)
        self.Kt_entry.grid(row=2, column=1)
        self.Kh_entry.grid(row=3, column=1)
        self.tauh_entry.grid(row=4, column=1)
        self.spool_entry.grid(row=5, column=1)
        self.t_end_entry.grid(row=6, column=1)
        self.dt_sim_entry.grid(row=7, column=1)

        self.J_entry.insert(0, "0.01")
        self.B_entry.insert(0, "0.1")
        self.Kt_entry.insert(0, "1.0")
        self.Kh_entry.insert(0, "0.2")
        self.tauh_entry.insert(0, "0.5")
        self.spool_entry.insert(0, "0.5")
        self.t_end_entry.insert(0, "5.0")
        self.dt_sim_entry.insert(0, "0.001")

        # 图表绘制风格选择区域
        style_frame = tk.LabelFrame(self, text="图表绘制风格", padx=10, pady=10)
        style_frame.grid(row=1, column=0, sticky="w", padx=10, pady=10)
        self.style_var = tk.StringVar(value="曲线图")
        style_options = ["线条图", "散点图", "填充图", "堆叠图", "对比强烈线条"]
        style_menu = tk.OptionMenu(style_frame, self.style_var, *style_options)
        style_menu.grid(row=1, column=1, sticky="w")

        # 图形显示区域
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312, sharex=self.ax1)
        self.ax3 = self.fig.add_subplot(313, sharex=self.ax1)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().grid(row=2, column=0, columnspan=3, padx=10, pady=10)

        # 按钮区域
        run_button = tk.Button(self, text="运行仿真", command=self.on_run, bg="#4CAF50", fg="white", font=("Arial", 12))
        run_button.grid(row=1, column=2, padx=10, pady=10)
        generate_button = tk.Button(self, text="生成图表", command=self.on_generate, bg="#FF9800", fg="white", font=("Arial", 12))
        generate_button.grid(row=1, column=3, padx=10, pady=10)
        save_button = tk.Button(self, text="保存图像", command=self.on_save, bg="#2196F3", fg="white", font=("Arial", 12))
        save_button.grid(row=1, column=4, padx=10, pady=10)
        save_data_button = tk.Button(self, text="保存数据", command=self.on_save_data, bg="#FFC107", fg="white", font=("Arial", 12))
        save_data_button.grid(row=1, column=5, padx=10, pady=10)

        # 进度条
        self.progress_label = tk.Label(self, text="计算进度：")
        self.progress_label.grid(row=3, column=0, padx=10, pady=10, sticky="w")
        self.progress = ttk.Progressbar(self, orient="horizontal", length=300, mode="determinate")
        self.progress.grid(row=3, column=1, padx=10, pady=10, sticky="w")

    def on_run(self):
        scenario = self.scenario_var.get()
        try:
            Kp = float(self.kp_entry.get())
            Ki = float(self.ki_entry.get())
            Kd = float(self.kd_entry.get())
        except ValueError:
            print("请输入有效的PID参数")
            return

        try:
            J = float(self.J_entry.get())
            B = float(self.B_entry.get())
            Kt = float(self.Kt_entry.get())
            K_h = float(self.Kh_entry.get())
            tau_h = float(self.tauh_entry.get())
            spool_max = float(self.spool_entry.get())
            t_end = float(self.t_end_entry.get())
            dt_sim = float(self.dt_sim_entry.get())
        except ValueError:
            print("请输入有效的模型参数")
            return

        self.progress["value"] = 0
        self.update_idletasks()

        # 调用仿真函数时，将所有参数传入
        time_list, phi_list, theta_list, phi_des_list = run_simulation(
            scenario, Kp, Ki, Kd,
            J, B, Kt, K_h, tau_h, spool_max,
            t_end, dt_sim, update_progress=self.update_progress
        )

        self.time_list = time_list
        self.phi_list = phi_list
        self.theta_list = theta_list
        self.phi_des_list = phi_des_list

        self.phi_deg = np.degrees(self.phi_list)
        self.theta_deg = np.degrees(self.theta_list)
        K_stiff = 1000.0
        self.F_h = K_stiff * np.array(self.phi_list)

        messagebox.showinfo("计算完成", "仿真计算已完成！请选择图表风格后点击生成图表。")

    def update_progress(self, value):
        self.progress["value"] = value
        self.update_idletasks()

    def on_generate(self):
        if not hasattr(self, 'time_list'):
            print("请先运行仿真！")
            return

        selected_style = self.style_var.get()

        # 清空图表
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        # 绘制第一张图——姿态角
        if selected_style == "线条图":
            self.ax1.plot(self.time_list, self.phi_deg, 'b-', label='姿态角', linewidth=2)
        elif selected_style == "散点图":
            self.ax1.scatter(self.time_list, self.phi_deg, color='blue', label='姿态角', s=30)
        elif selected_style == "填充图":
            self.ax1.fill_between(self.time_list, self.phi_deg, color='skyblue', alpha=0.5, label='姿态角')
        elif selected_style == "堆叠图":
            self.ax1.fill_between(self.time_list, self.phi_deg, color='lightgreen', label='姿态角')
        elif selected_style == "对比强烈线条":
            self.ax1.plot(self.time_list, self.phi_deg, 'k-', label='姿态角', linewidth=3)

        # 根据同一风格绘制第二张图——液压推力
        if selected_style == "线条图":
            self.ax2.plot(self.time_list, self.F_h, 'g-', label='液压推力', linewidth=2)
        elif selected_style == "散点图":
            self.ax2.scatter(self.time_list, self.F_h, color='green', label='液压推力', s=30)
        elif selected_style == "填充图":
            self.ax2.fill_between(self.time_list, self.F_h, color='lightgreen', alpha=0.5, label='液压推力')
        elif selected_style == "堆叠图":
            self.ax2.fill_between(self.time_list, self.F_h, color='lightblue', label='液压推力')
        elif selected_style == "对比强烈线条":
            self.ax2.plot(self.time_list, self.F_h, 'k-', label='液压推力', linewidth=3)

        # 根据同一风格绘制第三张图——阀芯角度
        if selected_style == "线条图":
            self.ax3.plot(self.time_list, self.theta_deg, 'm-', label='阀芯角度', linewidth=2)
        elif selected_style == "散点图":
            self.ax3.scatter(self.time_list, self.theta_deg, color='magenta', label='阀芯角度', s=30)
        elif selected_style == "填充图":
            self.ax3.fill_between(self.time_list, self.theta_deg, color='lightcoral', alpha=0.5, label='阀芯角度')
        elif selected_style == "堆叠图":
            self.ax3.fill_between(self.time_list, self.theta_deg, color='lightyellow', label='阀芯角度')
        elif selected_style == "对比强烈线条":
            self.ax3.plot(self.time_list, self.theta_deg, 'k-', label='阀芯角度', linewidth=3)

        # 设置轴标签和标题
        self.ax1.set_ylabel('姿态角 φ (°)', fontsize=12)
        self.ax1.set_title('智能钻头控制仿真', fontsize=14, fontweight='bold')
        self.ax2.set_ylabel('侧向推力 F_h (N)', fontsize=12)
        self.ax3.set_ylabel('阀芯角度 θ (°)', fontsize=12)
        self.ax3.set_xlabel('时间 t (s)', fontsize=12)

        # 添加网格
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax3.grid(True)

        self.canvas.draw()

    def on_save(self):
        file_path = filedialog.asksaveasfilename(title="保存图像",
                                                 filetypes=[("PNG Image", "*.png")],
                                                 defaultextension=".png")
        if file_path:
            self.fig.savefig(file_path)
            print(f"图像已保存: {file_path}")

    def on_save_data(self):
        file_path = filedialog.asksaveasfilename(title="保存数据",
                                                 filetypes=[("CSV File", "*.csv")],
                                                 defaultextension=".csv")
        if file_path:
            with open(file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Time (s)', '姿态角 φ (°)', '阀芯角度 θ (°)', '侧向推力 F_h (N)'])
                for t, phi, theta, F in zip(self.time_list, self.phi_deg, self.theta_deg, self.F_h):
                    writer.writerow([t, phi, theta, F])
            print(f"数据已保存: {file_path}")

if __name__ == "__main__":
    app = DrillSimUI()
    app.mainloop()
