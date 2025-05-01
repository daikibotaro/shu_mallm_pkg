import cv2
import os
import json
import re
import tkinter as tk
from screeninfo import get_monitors
from rich import print


class DisplayManager:
    def __init__(self):
        self.root = None
        self.history_file = "/home/hiratalab/catkin_ws/src/shu_task_planning/history/message_history.json"

    def show_image(self, image_filename):
        # Check if the file exists
        if os.path.exists(image_filename):
            # Load and display the image using OpenCV
            img = cv2.imread(image_filename)
            cv2.imshow('Image', img)
            cv2.waitKey(0)  # Wait for a key press to close the window
            cv2.destroyAllWindows()
            return True
        else:
            print(f"File {image_filename} does not exist.")
            return False

    def create_window(self, window):
        monitors = get_monitors()
        target_monitor = monitors[0]
        window_width = target_monitor.width
        window_height = target_monitor.height
        window.geometry(f"{window_width}x{window_height}+{target_monitor.x}+{target_monitor.y}")
        window.configure(bg='black')
        return window_width, window_height

    def create_root(self):
        self.root = tk.Tk()
        self.root.title("ROS window")
        self.window_width, self.window_height = self.create_window(self.root)

    def destroy_root(self):
        if self.root is not None:
            self.root.destroy()
            self.root = None

    def check_task_sequence(self):
        task_window = tk.Toplevel(self.root)  # 新しいウィンドウを作成
        task_window.title("Task Check Window")
        window_width, window_height = self.create_window(task_window)  # 画面いっぱいに設定

        # 会話履歴ファイル読み込み
        with open(self.history_file, 'r') as file:
            message_history = json.load(file)
        length = len(message_history)
        # 会話履歴からタスクの手順を抽出
        task_sequence = message_history[int(length - 3)]["content"]
        # 正規表現を使って手順を抽出 (1.~~~ 2.~~~ 3.~~~)
        pattern = r"(^\d\.\s.*|^\s*-\s.*$)"
        matches = re.findall(pattern, task_sequence, re.MULTILINE)
        extracted_text = "\n".join(matches).replace("-", "・")  # 手順を表示用に整形

        # 手順を表示するラベルを作成し、中央に配置
        task_label = tk.Label(task_window, text=extracted_text, bg='black', fg='white', font=("Arial", 30), wraplength=window_width * 0.8, justify="left")
        task_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        # 閉じるボタンを作成し、ウィンドウ下部に配置
        close_button = tk.Button(task_window, text="閉じる", command=task_window.destroy, font=("Helvetica", 50), width=8, height=2, bg='white', fg='black')
        close_button.place(relx=0.5, rely=0.9, anchor=tk.CENTER)

    def display_message(self, message):
        self.create_root()
        font_size = 70
        font = ("Arial", font_size)
        label = tk.Label(self.root, text=message, bg='black', fg='white', font=font)
        label.pack(pady=40)
        label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        self.root.update_idletasks()
        while label.winfo_width() + 100 > self.window_width and font_size > 10:
            font_size -= 1
            font = ("Arial", font_size)
            label.config(font=font)
            self.root.update_idletasks()
        close_button = tk.Button(self.root, text="閉じる", command=self.root.quit, font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        close_button.place(relx=0.5, rely=0.7, anchor=tk.CENTER)

        # 確認ボタンを作成
        hello_button = tk.Button(self.root, text="タスクを確認", command=self.check_task_sequence, font=("Helvetica", 50), bg='blue', fg='white')
        hello_button.place(relx=0.95, rely=0.95, anchor=tk.SE)

        self.root.mainloop()
        self.destroy_root()

    def wait_for_user(self):
        self.create_root()

        def close_window():
            self.root.quit()
        close_button = tk.Button(self.root, text="待機終了", command=close_window, font=("Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        close_button.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        # タスク確認ボタンを作成
        hello_button = tk.Button(self.root, text="タスクを確認", command=self.check_task_sequence, font=("Helvetica", 50), bg='blue', fg='white')
        hello_button.place(relx=0.95, rely=0.95, anchor=tk.SE)

        self.root.mainloop()
        self.destroy_root()

    def ask_yes_or_no(self, question):
        self.create_root()
        status = False
        font_size = 70
        font = ("Arial", font_size)
        label = tk.Label(self.root, text=question, bg='black', fg='white', font=font)
        label.pack(pady=40)
        label.place(relx=0.5, rely=0.4, anchor=tk.CENTER)
        self.root.update_idletasks()
        while label.winfo_width() + 50 > self.window_width and font_size > 10:
            font_size -= 1
            font = ("Arial", font_size)
            label.config(font=font)
            self.root.update_idletasks()

        def close_window(response):
            nonlocal status
            status = response
            self.root.quit()
        yes_button = tk.Button(self.root, text="はい", command=lambda: close_window(True), font=("Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        no_button = tk.Button(self.root, text="いいえ", command=lambda: close_window(False), font=("Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        yes_button.place(relx=0.3, rely=0.7, anchor=tk.CENTER)
        no_button.place(relx=0.7, rely=0.7, anchor=tk.CENTER)

        # タスク確認ボタンを作成
        hello_button = tk.Button(self.root, text="タスクを確認", command=self.check_task_sequence, font=("Helvetica", 50), bg='blue', fg='white')
        hello_button.place(relx=0.95, rely=0.95, anchor=tk.SE)

        self.root.mainloop()
        self.destroy_root()
        print(status)
        return status

    def ask_question(self, question):
        self.create_root()
        font_size = 70
        font = ("Arial", font_size)
        answer = ""

        def close_window(response):
            nonlocal answer
            if response == "OK":
                answer = entry.get()
                print(answer)
            elif response == "キャンセル":
                answer = None
                print("キャンセルされました")
            self.root.quit()

        label = tk.Label(self.root, text=question, bg='black', fg='white', font=font)
        label.pack(pady=40)
        label.place(relx=0.5, rely=0.3, anchor=tk.CENTER)

        entry = tk.Entry(self.root, font=font, width=30)
        entry.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        entry.focus_set()  # 入力欄にフォーカスを設定

        ok_button = tk.Button(self.root, text="OK", command=lambda: close_window("OK"), font=("Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        cancel_button = tk.Button(self.root, text="キャンセル", command=lambda: close_window("キャンセル"), font=("Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        ok_button.place(relx=0.3, rely=0.7, anchor=tk.CENTER)
        cancel_button.place(relx=0.7, rely=0.7, anchor=tk.CENTER)

        # タスク確認ボタンを作成
        hello_button = tk.Button(self.root, text="タスクを確認", command=self.check_task_sequence, font=("Helvetica", 50), bg='blue', fg='white')
        hello_button.place(relx=0.95, rely=0.95, anchor=tk.SE)

        self.root.mainloop()
        self.destroy_root()

        return answer

    def select_option(self, question, option1, option2):
        self.create_root()
        selected_option = None
        font_size = 70
        font = ("Arial", font_size)

        # 質問を表示
        label = tk.Label(self.root, text=question, bg='black', fg='white', font=font)
        label.pack(pady=40)
        label.place(relx=0.5, rely=0.4, anchor=tk.CENTER)
        self.root.update_idletasks()

        # フォントサイズを調整
        while label.winfo_width() + 50 > self.window_width and font_size > 10:
            font_size -= 1
            font = ("Arial", font_size)
            label.config(font=font)
            self.root.update_idletasks()

        # オプション1のボタンを作成
        def close_window(option):
            nonlocal selected_option
            selected_option = option
            self.root.quit()

        option1_button = tk.Button(self.root, text=option1, command=lambda: close_window(option1), font=("Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        option1_button.place(relx=0.3, rely=0.7, anchor=tk.CENTER)

        # オプション2のボタンを作成
        option2_button = tk.Button(self.root, text=option2, command=lambda: close_window(option2), font=("Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        option2_button.place(relx=0.7, rely=0.7, anchor=tk.CENTER)

        # タスク確認ボタンを作成
        hello_button = tk.Button(self.root, text="タスクを確認", command=self.check_task_sequence, font=("Helvetica", 50), bg='blue', fg='white')
        hello_button.place(relx=0.95, rely=0.95, anchor=tk.SE)

        self.root.mainloop()
        self.destroy_root()

        print(selected_option)
        return selected_option
