    def ask_two_options(self, question, option1, option2):
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

        self.root.mainloop()
        self.destroy_root()

        print(selected_option)
        return selected_option