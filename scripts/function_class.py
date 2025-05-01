#!/usr/bin/env python3

import rospy
from openai import OpenAI
import base64
from geometry_msgs.msg import PoseStamped
import actionlib
import subprocess
import math
import cv2
import json
import os
import tkinter as tk
from screeninfo import get_monitors
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from playsound import playsound
import datetime
import re
from rich import print


class RobotController:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
        self.save_image_dir = "/home/hiratalab/catkin_ws/src/shu_task_planning/images/"

    def move_to_point_xy(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            result = self.client.get_result()
            if result:
                rospy.loginfo("Goal reached!!!")
                return True
            else:
                rospy.logerr(f"Goal failed")
                return False

    def move_to_point(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(theta * math.pi / 360)
        goal.target_pose.pose.orientation.w = math.cos(theta * math.pi / 360)
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            result = self.client.get_result()
            if result:
                rospy.loginfo("Goal reached!!!")
                return True
            else:
                rospy.logerr(f"Goal failed")
                return False

    def move_to_location(self, place):
        # 場所の名前と対応する座標の辞書
        locations = {
            "事務所": (0.032, -0.025, 0.0),
            "詰所": (0.000, -4.980, 0.0),
            "部品棚": (-0.844, 6.307, 0.0),
            "受入れ": (-2.868, 8.319, 0.0),
            "組立て工程": (1.916, 2.294, 0.0),
            "塗装工程": (-2.307, -2.545, 0.0),
            "検査工程": (6.020, -0.776, 0.0)
        }

        # 指定された場所が辞書に存在するか確認
        if place in locations:
            # 場所の名前に対応する座標を取得
            x, y, theta = locations[place]
            # move_to_point関数を呼び出してロボットを移動
            result = self.move_to_point(x, y, theta)
            return result
        else:
            rospy.logerr("指定された場所は存在しません: {}".format(place))
            return None

    def get_robot_position(self):
        # rospy.init_node('robot_position_listener', anonymous=True)

        # Wait for one message from the topic and store it
        pose_msg = rospy.wait_for_message('/robot_map_position', PoseStamped)

        # Print the received message
        pos_x, pos_y = pose_msg.pose.position.x, pose_msg.pose.position.y

        ori_z, ori_w = pose_msg.pose.orientation.z, pose_msg.pose.orientation.w
        theta = math.atan2(ori_z, ori_w) * 360 / math.pi

        print("Received PoseStamped message:")
        print(f"Position:")
        print(f"  x: {pose_msg.pose.position.x}")
        print(f"  y: {pose_msg.pose.position.y}")
        print(f"  z: {pose_msg.pose.position.z}")
        print(f"Orientation:")
        print(f"  x: {pose_msg.pose.orientation.x}")
        print(f"  y: {pose_msg.pose.orientation.y}")
        print(f"  z: {pose_msg.pose.orientation.z}")
        print(f"  w: {pose_msg.pose.orientation.w}")

        print(f"\ntheta: {theta}")

        return (pos_x, pos_y, theta)

    def return_to_initial_position(self, initial_position):
        x, y, theta = initial_position
        result = self.move_to_point(x, y, theta)
        return result

    def take_photo(self):
        cap = cv2.VideoCapture('/dev/video0')
        if not cap.isOpened():
            rospy.logerr("Camera device not found")
            return
        try:
            ret, frame = cap.read()
            if not ret:
                rospy.logerr("Failed to capture image from camera")
                return
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(image_msg)
            rospy.loginfo("Photo captured")
            rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)
            window_name = 'Captured Image'
            cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow(window_name, rotated_frame)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

            if not os.path.exists(self.save_image_dir):
                os.makedirs(self.save_image_dir)
            get_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            image_filename = os.path.join(self.save_image_dir, f"{get_time}.jpg")
            cv2.imwrite(image_filename, rotated_frame)
            rospy.loginfo(f"saved as {image_filename}")
            return image_filename
        except Exception as e:
            rospy.logerr(f"Error occurred: {e}")
            return None
        finally:
            cap.release()


class LLMProcessor:
    def __init__(self):
        self.client = OpenAI()
        self.history_file = "/home/hiratalab/catkin_ws/src/shu_task_planning/history/message_history.json"

    def generate_response(self, text, model):
        response = self.client.chat.completions.create(
            model=model,
            messages=[
                {"role": "user", "content": text}
            ],
        )
        print(response.choices[0].message.content)
        return response.choices[0].message.content

    def create_message_dictionary(self, role, content):
        return {"role": role, "content": content}

    def save_message_history(self, message_history):
        file_path = self.history_file
        with open(file_path, 'w') as file:
            json.dump(message_history, file, ensure_ascii=False, indent=4)
        # print(f"Message history saved to {file_path}")

    def add_message_history(self, role, content):
        file_path = self.history_file

        with open(file_path, 'r') as file:
            message_history = json.load(file)
        # print(f"Message history loaded from {file_path}")

        new_dic = self.create_message_dictionary(role, content)

        message_history.append(new_dic)

        self.save_message_history(message_history)

    def load_message_history(self):
        file_path = self.history_file
        with open(file_path, 'r') as file:
            message_history = json.load(file)
        # print(f"Message history loaded from {file_path}")
        return message_history

    def generate_response_history(self, text):
        messages = self.load_message_history()
        messages.append(self.create_message_dictionary("user", text))
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=messages
        )
        print(response.choices[0].message.content)
        return response.choices[0].message.content

    def encode_image(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    def AI_vision(self, image_filename, text):
        add = "回答は必ず日本語で行ってください．出力に句読点以外の記号を含めずに回答してください．"
        base64_image = self.encode_image(image_filename)
        message_text = {"type": "text", "text": text + add}
        message_image = {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/png;base64,{base64_image}",
                "detail": "low"
            }
        }
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "user", "content": [message_text, message_image]}
            ],
            # temperature=0.0,
        )
        # chat_profile = cl.user_session.get("chat_profile")
        # cl.Message(content=f"starting chat using the {chat_profile} chat profile").send()

        explanation = response.choices[0].message.content
        completion_tokens, prompt_usage, total_usage = response.usage.completion_tokens, response.usage.prompt_tokens, response.usage.total_tokens
        print(explanation)
        print(f"\ncompletion_tokens: {completion_tokens}  prompt_tokens: {prompt_usage}  total usage: {total_usage}")

        self.add_message_history("user", [message_text, message_image])
        self.add_message_history("assistant", explanation)

        return explanation


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

        with open(self.history_file, 'r') as file:
            message_history = json.load(file)
        length = len(message_history)
        # 手順の部分を会話履歴から引っ張ってくる
        task_sequence = message_history[int(length - 3)]["content"]
        # パターン設定(1.~~~ 2.~~~ 3.~~~)正規表現
        pattern = r"(^\d\.\s.*|^\s*-\s.*$)"
        # マッチング&取り出し
        matches = re.findall(pattern, task_sequence, re.MULTILINE)
        # リスト統合
        extracted_text = "\n".join(matches)
        # message = extracted_text.replace(".", " ")
        extracted_text = extracted_text.replace("-", "・")
        # 手順の表示
        task_label = tk.Label(task_window, text=extracted_text, bg='black', fg='white', font=("Arial", 30), wraplength=window_width * 0.8, justify="left")
        task_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)  # 画面中央に表示

        # 閉じるボタンを追加
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

        # 確認ボタンの追加
        hello_button = tk.Button(self.root, text="タスクを確認", command=self.check_task_sequence, font=("Helvetica", 50), bg='blue', fg='white')
        hello_button.place(relx=0.95, rely=0.95, anchor=tk.SE)

        self.root.mainloop()
        self.destroy_root()

    def wait_for_user(self):
        self.create_root()

        def close_window():
            self.root.quit()
        close_button = tk.Button(self.root, text="待機終了", command=close_window, font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        close_button.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        # 確認ボタンの追加
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
        yes_button = tk.Button(self.root, text="はい", command=lambda: close_window(True), font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        no_button = tk.Button(self.root, text="いいえ", command=lambda: close_window(False), font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        yes_button.place(relx=0.3, rely=0.7, anchor=tk.CENTER)
        no_button.place(relx=0.7, rely=0.7, anchor=tk.CENTER)

        # 確認ボタンの追加
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
        entry.focus_set()  # 入力欄に初期フォーカスを設定

        ok_button = tk.Button(self.root, text="OK", command=lambda: close_window("OK"), font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        cancel_button = tk.Button(self.root, text="キャンセル", command=lambda: close_window("キャンセル"), font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        ok_button.place(relx=0.3, rely=0.7, anchor=tk.CENTER)
        cancel_button.place(relx=0.7, rely=0.7, anchor=tk.CENTER)

        # Tabキーでボタン間を移動できるようにするコード
        ok_button.bind("<Tab>", lambda e: cancel_button.focus_set())
        cancel_button.bind("<Tab>", lambda e: ok_button.focus_set())

        # Enterキーでフォーカスされたボタンをクリックするコード
        def on_enter(event):
            focused_widget = self.root.focus_get()
            if focused_widget == ok_button:
                close_window("OK")
            elif focused_widget == cancel_button:
                close_window("キャンセル")

        self.root.bind("<Return>", on_enter)

        # 確認ボタンの追加
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

        # 質問の表示
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

        def close_window(option):
            nonlocal selected_option
            selected_option = option
            self.root.quit()

        # オプション1のボタンを作成
        option1_button = tk.Button(self.root, text=option1, command=lambda: close_window(option1), font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        option1_button.place(relx=0.3, rely=0.7, anchor=tk.CENTER)

        # オプション2のボタンを作成
        option2_button = tk.Button(self.root, text=option2, command=lambda: close_window(option2), font=(
            "Helvetica", 50), width=10, height=2, bg='white', fg='black', relief=tk.FLAT)
        option2_button.place(relx=0.7, rely=0.7, anchor=tk.CENTER)

        # 確認ボタンの追加
        hello_button = tk.Button(self.root, text="タスクを確認", command=self.check_task_sequence, font=("Helvetica", 50), bg='blue', fg='white')
        hello_button.place(relx=0.95, rely=0.95, anchor=tk.SE)

        self.root.mainloop()
        self.destroy_root()

        print(selected_option)
        return selected_option


class TextToSpeech:
    def __init__(self):
        self.bt_mac_address = '40:72:18:41:8E:C6'

        self.mecabDicPath = '/var/lib/mecab/dic/open-jtalk/naist-jdic'
        self.voice_model = '/usr/share/hts-voice/mei/mei_angry.htsvoice'
        # self.voice_model = '/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice'

        self.text_file = '/home/hiratalab/catkin_ws/src/shu_task_planning/voice/temp.txt'
        self.audio_file = '/home/hiratalab/catkin_ws/src/shu_task_planning/voice/output.wav'
        self.silence_file = '/home/hiratalab/catkin_ws/src/shu_task_planning/voice/silence.wav'
        self.final_output = '/home/hiratalab/catkin_ws/src/shu_task_planning/voice/final_output.wav'

        self.talk_speed = 0.8

    def talk(self, text):
        with open(self.text_file, "w") as f:
            f.write(text)
        command = ['open_jtalk']
        command += ['-x', self.mecabDicPath]
        command += ['-m', self.voice_model]
        command += ['-ow', self.audio_file]
        command += ['-r', str(self.talk_speed)]
        # with open('/home/hiratalab/catkin_ws/src/shu_task_planning/voice/temp.txt', 'w') as f:
        # f.write(text)
        command += [self.text_file]
        subprocess.run(command)
        # subprocess.run(['sox', '-n', '-r', '48000', '-c', '1', self.silence_file, 'trim', '0.0', '2.0'])
        subprocess.run(['sox', self.silence_file, self.audio_file, self.final_output])
        # subprocess.run(['aplay', '/home/hiratalab/catkin_ws/src/shu_task_planning/voice/output.wav'])
        start = datetime.datetime.now()
        start_time = f"{start.hour}:{start.minute}:{start.second}.{int(start.microsecond / 10000):02d}"
        print("再生開始", start_time)
        self.play_sound(self.final_output)
        end = datetime.datetime.now()
        end_time = f"{end.hour}:{end.minute}:{end.second}.{int(end.microsecond / 10000):02d}"
        print("再生終了", end_time)
        duration = (end - start).total_seconds()
        print(f"再生時間: {duration} 秒")
        return True

    def play_sound(self, file_path):
        playsound(file_path)

    def get_sink_by_name(self, name):
        result = subprocess.run(['pactl', 'list', 'short', 'sinks'], stdout=subprocess.PIPE)
        sinks = result.stdout.decode().split("\n")
        for sink in sinks:
            if name in sink:
                return sink.split("\t")[0]
        return None

    def set_default_sink(self, sink_name):
        subprocess.run(["pactl", "set-default-sink", sink_name])

    def connect_to_bluetooth_speaker(self, mac_address):
        connect_command = f"bluetoothctl connect {mac_address}"
        try:
            subprocess.run(connect_command, shell=True, check=True)
            print(f"スピーカー {mac_address} に接続しました。")
        except subprocess.CalledProcessError as e:
            print(f"スピーカー {mac_address} への接続に失敗しました: {e}")

    def text_to_speech(self, content, speaker_number=0):
        # self.talk(content)
        if speaker_number == 0:
            internal_speaker_sink = self.get_sink_by_name("alsa_output")
            if internal_speaker_sink:
                print("デフォルトの音声出力を内蔵スピーカーに変更しました。")
                self.set_default_sink(internal_speaker_sink)
                # time.sleep(1)
                self.talk(content)
                # self.play_sound('output.wav')
            else:
                print("内蔵スピーカーのsinkが見つかりませんでした。")

        elif speaker_number == 1:
            self.connect_to_bluetooth_speaker(self.bt_mac_address)
            bluetooth_sink = self.get_sink_by_name("bluez_sink")
            if bluetooth_sink:
                self.set_default_sink(bluetooth_sink)
                print("デフォルトの音声出力をBluetoothスピーカーに変更しました。")
                # time.sleep(1)
                self.talk(content)

                # self.play_sound('output.wav')
            else:
                print("Bluetoothスピーカーのsinkが見つかりませんでした。")

        elif speaker_number == 2:
            usb_speaker_sink = self.get_sink_by_name("alsa_output.usb")
            if usb_speaker_sink:
                self.set_default_sink(usb_speaker_sink)
                print("デフォルトの音声出力をUSBスピーカーに変更しました。")
                self.talk(content)
            else:
                print("USBスピーカーのsinkが見つかりませんでした。")
        else:
            print("無効なスピーカー番号が指定されました。")

    def play_message_internal(self, message):
        self.text_to_speech(message, 0)

    def play_message_bluetooth(self, message):
        self.text_to_speech(message, 1)

    def play_message_usb(self, message):
        self.text_to_speech(message, 2)


if __name__ == '__main__':
    # rospy.init_node('test')
    # controller = RobotController()
    # llm = LLMProcessor()
    # history = llm.load_message_history()
    # print(history)
    # llm.add_message_history({"role": "user", "content": "はい"})
    # prompt = "これは何？"
    # img = controller.take_photo()
    # explanation = llm.AI_vision(img, prompt)
    # print(explanation)
    # DisplayManager クラスのインスタンスを作成
    display_manager = DisplayManager()

    # 質問と2つの選択肢を定義
    question = "あなたが好きなのは？"
    option1 = "犬"
    option2 = "猫"

    # ask_two_options 関数を呼び出してユーザーの選択を取得
    selected_option = display_manager.select_option(question, option1, option2)

    # 選択された結果を表示
    print(f"選択されたのは: {selected_option} です。")
