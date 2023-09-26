#!/usr/bin/env python3
import rospy
import rospkg

import tkinter as tk
import openai
from utils import post_processing, subtree_assembly
import time

class Pipeline:
    def __init__(self, generate_file, prompt_dir, sub_tree_dir, api_key, model_name):

        self.generate_file = generate_file
        self.prompt_dir = prompt_dir
        self.sub_tree_dir = sub_tree_dir
        self.api_key = api_key
        self.model_name = model_name

        
        
        self.status_environment_descriptions = False
        self.status_robot_sensing = False
        self.status_source_bt = False
        self.status_existing_subtree = False
        self.input_task = ""

        self.input_environment_descriptions = ""
        self.input_robot_sensing = ""
        self.input_source_bt = ""
        self.input_existing_subtree = ""

        self.text_environment_descriptions = ""
        self.text_robot_sensing = ""
        self.text_source_bt = ""
        self.text_existing_subtree = ""

        self.final_prompt = ""

        self.options = ["Enable", "Disable"]
        
        self.window = tk.Tk()
        self.window.title("Behavior Tree Generation")
        self.window.geometry('1400x1000')

        self.upper_frame = tk.Frame(self.window)
        self.middle_frame = tk.Frame(self.window)
        self.lower_frame = tk.Frame(self.window)

        self.task_label = tk.Label(self.upper_frame, text="Enter target task description:")
        self.task_label.config(font=('Helvatical bold',20))

        self.text = tk.Text(self.upper_frame, height=3, width=80, font=("Arial", 20))

        self.ed_label = tk.Label(self.middle_frame, text="Prompts about environment descriptions: ")
        self.ed_label.config(font=('Helvatical bold',20))

        self.ed_variable = tk.StringVar(self.middle_frame)
        self.ed_variable.set(self.options[1])
        self.ed_selector = tk.OptionMenu(self.middle_frame, self.ed_variable, *self.options)
        self.ed_selector.config(height=2, width=15, font=('Helvatical bold',20)) 

        self.rs_label = tk.Label(self.middle_frame, text="Prompts about robot sensing abilities: ")
        self.rs_label.config(font=('Helvatical bold',20))

        self.rs_variable = tk.StringVar(self.middle_frame)
        self.rs_variable.set(self.options[1])
        self.rs_selector = tk.OptionMenu(self.middle_frame, self.rs_variable, *self.options)
        self.rs_selector.config(height=2, width=15, font=('Helvatical bold',20)) 

        self.sb_label = tk.Label(self.middle_frame, text="Prompts about source behavior trees: ")
        self.sb_label.config(font=('Helvatical bold',20))

        self.sb_variable = tk.StringVar(self.middle_frame)
        self.sb_variable.set(self.options[1])
        self.sb_selector = tk.OptionMenu(self.middle_frame, self.sb_variable, *self.options)
        self.sb_selector.config(height=2, width=15, font=('Helvatical bold',20)) 

        self.es_label = tk.Label(self.middle_frame, text="Prompts about existing sub trees: ")
        self.es_label.config(font=('Helvatical bold',20))
        self.es_variable = tk.StringVar(self.middle_frame)
        self.es_variable.set(self.options[1])
        self.es_selector = tk.OptionMenu(self.middle_frame, self.es_variable, *self.options)
        self.es_selector.config(height=2, width=15, font=('Helvatical bold',20)) 

        self.ed_label_2 = tk.Label(self.middle_frame, text="Current prompt about environment descriptions:")
        self.ed_label_2.config(font=('Helvatical bold',20))
        
        
        self.rs_label_2 = tk.Label(self.middle_frame, text="Current prompt about robot sensing abilities:")
        self.rs_label_2.config(font=('Helvatical bold',20))

        self.sb_label_2 = tk.Label(self.middle_frame, text="Current prompt about source behavior trees:")
        self.sb_label_2.config(font=('Helvatical bold',20))

        
        self.es_label_2 = tk.Label(self.middle_frame, text="Current prompt about existing sub trees:")
        self.es_label_2.config(font=('Helvatical bold',20))

        
        self.task_button = tk.Button(self.lower_frame, text="Submit", command=self.task_submit)
        self.task_button.config(height=2, width=15, font=('Helvatical bold',20)) 


        self.view_prompt_ed_button = tk.Button(self.middle_frame, text="View detail and edit", command=self.view_prompt_ed)
        self.view_prompt_ed_button.config(height=2, width=15, font=('Helvatical bold',20)) 
        self.view_prompt_rs_button = tk.Button(self.middle_frame, text="View detail and edit", command=self.view_prompt_rs)
        self.view_prompt_rs_button.config(height=2, width=15, font=('Helvatical bold',20)) 
        self.view_prompt_sb_button = tk.Button(self.middle_frame, text="View detail and edit", command=self.view_prompt_sb)
        self.view_prompt_sb_button.config(height=2, width=15, font=('Helvatical bold',20)) 
        self.view_prompt_es_button = tk.Button(self.middle_frame, text="View detail and edit", command=self.view_prompt_es)
        self.view_prompt_es_button.config(height=2, width=15, font=('Helvatical bold',20)) 

    def ui_pack(self):
        self.upper_frame.pack()
        self.task_label.pack()
        self.text.pack()

        self.middle_frame.pack()
        self.ed_label.grid(row=0, column=0, padx=5, pady=5)
        self.ed_selector.grid(row=1, column=0, padx=5, pady=5)
        self.rs_label.grid(row=2, column=0, padx=5, pady=5)
        self.rs_selector.grid(row=3, column=0, padx=5, pady=5)
        self.sb_label.grid(row=4, column=0, padx=5, pady=5)
        self.sb_selector.grid(row=5, column=0, padx=5, pady=5)
        self.es_label.grid(row=6, column=0, padx=5, pady=5)
        self.es_selector.grid(row=7, column=0, padx=5, pady=5)
        self.ed_label_2.grid(row=0, column=1, padx=5, pady=5)
        self.view_prompt_ed_button.grid(row=1, column=1, padx=5, pady=5)
        self.rs_label_2.grid(row=2, column=1, padx=5, pady=5)
        self.view_prompt_rs_button.grid(row=3, column=1, padx=5, pady=5)
        self.sb_label_2.grid(row=4, column=1, padx=5, pady=5)
        self.view_prompt_sb_button.grid(row=5, column=1, padx=5, pady=5)
        self.es_label_2.grid(row=6, column=1, padx=5, pady=5)
        self.view_prompt_es_button.grid(row=7, column=1, padx=5, pady=5)

        self.lower_frame.pack()
        self.task_button.pack()

    def get_single_prompt(self, root_dir, file_name):
        file_path = root_dir + file_name
        # print(file_path)
        prompt = ""
        with open(file_path, 'r') as file:
            prompt = file.read().rstrip()
        return prompt

    def input_process(self, input):
        if input == "Enable":
            return True
        elif input == "Disable":
            return False
        else:
            raise NotImplementedError

    def LLM_generation(self):
        openai.api_key = self.api_key
        response = openai.Completion.create(
            model=self.model_name,
            prompt=self.final_prompt,
            temperature=0,
            max_tokens=500,
            top_p=1,
            presence_penalty=0,
            frequency_penalty=0.2
        )

        raw_string = str(response['choices'][0]['text'])



        raw_path = self.generate_file + "raw.txt"
        full_path = self.generate_file + "test.tree"

        raw_text_file = open(raw_path, "wt")
        n = raw_text_file.write(raw_string)
        raw_text_file.close()

        output_text = post_processing(raw_string)



        text_file = open(full_path, "wt")
        m = text_file.write(subtree_assembly(output_text, self.sub_tree_dir))
        text_file.close()


    def view_prompt_ed(self):
        def on_popup_window_close():
            self.input_environment_descriptions = text_label.get("1.0", tk.END).strip() + "\n"
            popup_window.destroy()
        
        popup_window = tk.Toplevel(self.window)
        popup_window.title("Prompt about environment descriptions")

        text_label = tk.Text(popup_window, height=30, width=120, font=("Arial", 20))
        text_label.pack()

        save_button = tk.Button(popup_window, text="Save and Close", command=on_popup_window_close)
        save_button.config(height=2, width=15, font=('Helvatical bold',20)) 
        save_button.pack()
        
        text_label.insert(tk.END, self.get_single_prompt(self.prompt_dir, "environment_description.txt"))

        

    def view_prompt_es(self):
        def on_popup_window_close():
            self.input_existing_subtree = text_label.get("1.0", tk.END).strip() + "\n"
            popup_window.destroy()

        popup_window = tk.Toplevel(self.window)
        popup_window.title("Prompt about existing sub trees")

        text_label = tk.Text(popup_window, height=30, width=120, font=("Arial", 20))
        text_label.pack()

        save_button = tk.Button(popup_window, text="Save and Close", command=on_popup_window_close)
        save_button.config(height=2, width=15, font=('Helvatical bold',20)) 
        save_button.pack()

        text_label.insert(tk.END, self.get_single_prompt(self.prompt_dir, "existing_sub_trees.txt"))


    def view_prompt_rs(self):
        def on_popup_window_close():
            self.input_robot_sensing = text_label.get("1.0", tk.END).strip() + "\n"
            popup_window.destroy()

        popup_window = tk.Toplevel(self.window)
        popup_window.title("Prompt about robot sensing abilities")

        text_label = tk.Text(popup_window, height=30, width=120, font=("Arial", 20))
        text_label.pack()

        save_button = tk.Button(popup_window, text="Save and Close", command=on_popup_window_close)
        save_button.config(height=2, width=15, font=('Helvatical bold',20)) 
        save_button.pack()

        text_label.insert(tk.END, self.get_single_prompt(self.prompt_dir, "robot_sensing_abilities.txt"))

        

    def view_prompt_sb(self):
        def on_popup_window_close():
            self.input_source_bt = text_label.get("1.0", tk.END).strip() + "\n"
            popup_window.destroy()

        popup_window = tk.Toplevel(self.window)
        popup_window.title("Prompt about existing sub trees")

        text_label = tk.Text(popup_window, height=30, width=120, font=("Arial", 20))
        text_label.pack()

        save_button = tk.Button(popup_window, text="Save and Close", command=on_popup_window_close)
        save_button.config(height=2, width=15, font=('Helvatical bold',20)) 
        save_button.pack()

        text_label.insert(tk.END, self.get_single_prompt(self.prompt_dir, "source_behavior_tree.txt"))


    def task_submit(self):
        self.status_environment_descriptions = self.input_process(self.ed_variable.get())
        self.status_robot_sensing = self.input_process(self.rs_variable.get())
        self.status_source_bt = self.input_process(self.sb_variable.get())
        self.status_existing_subtree = self.input_process(self.es_variable.get())


        self.input_task = self.text.get("1.0", tk.END).strip()

        if self.status_environment_descriptions == False:
            self.input_environment_descriptions = ""

        if self.status_robot_sensing == False:  
            self.input_robot_sensing = ""

        if self.status_source_bt == False:
            self.input_source_bt = ""

        if self.status_existing_subtree == False:
            self.input_existing_subtree = ""
        

        self.final_prompt = self.input_environment_descriptions + \
                            self.input_robot_sensing + \
                            self.input_existing_subtree + \
                            self.input_source_bt + \
                            self.input_task 

        print(self.final_prompt)
        # call api
        self.LLM_generation()

        self.task_button.config(text="Finished")

        trigger_rosparam = rospy.get_param('~trigger_rosparam', '')

        rospy.set_param(trigger_rosparam, True)


    def mainloop(self):
        self.window.mainloop()


if __name__ == '__main__':
    rospy.init_node('behavior_tree_generation_node')
    rospack = rospkg.RosPack()

    generate_file = rospack.get_path('behavior_tree') + "/config/drone_latching/"
    prompt_dir = rospack.get_path('behavior_tree_generation') + "/config/prompt/"
    sub_tree_dir = rospack.get_path('behavior_tree_generation') + "/config/subtree/"
    api_key = rospy.get_param('~api_key', '')
    trigger_rosparam = rospy.get_param('~trigger_rosparam', '')
    model_name = rospy.get_param('~model_name', '')

    print(api_key)
   
    rospy.set_param(trigger_rosparam, False)

    bt_generation = Pipeline(
        generate_file, 
        prompt_dir, 
        sub_tree_dir, 
        api_key, 
        model_name
    )
    bt_generation.ui_pack()
    bt_generation.mainloop()

    

    


    


