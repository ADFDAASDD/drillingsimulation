# drillingsimulation
import tkinter as tk
from tkinter import ttk
import ui
class MainInterface(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("主界面")
        self.geometry("600x400")
        self.create_widgets()

    def create_widgets(self):
        # 标题标签
        title_label = tk.Label(self, text="请选择进入的子模块", font=("Arial", 16))
        title_label.pack(pady=20)

        # 按钮进入“智能钻头控制仿真平台”
        sim_button = tk.Button(self, text="智能钻头控制仿真平台", font=("Arial", 14),
                               width=30, height=2, command=self.open_simulation)
        sim_button.pack(pady=10)

        # 如果后续有其他模块，可以继续添加按钮
        # 例如：
        # another_button = tk.Button(self, text="其他模块", font=("Arial", 14),
        #                            width=30, height=2, command=self.open_another_module)
        # another_button.pack(pady=10)

    def open_simulation(self):
        # 关闭主界面后，进入子模块（仿真界面）
        self.destroy()
        # 创建并启动智能钻头控制仿真平台
        sim_app = ui.DrillSimUI()
        sim_app.mainloop()


if __name__ == "__main__":
    app = MainInterface()
    app.mainloop()
