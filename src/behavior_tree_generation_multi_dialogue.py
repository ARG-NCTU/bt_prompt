#!/usr/bin/env python3
import json
import os

import openai
import rospy


class Args:
    def __init__(self, dictionary):
        for key, value in dictionary.items():
            setattr(self, key, value)

    def __str__(self):
        return str(self.__dict__)


class MultiDialogue:
    def __init__(self, args):
        # initialize arguments
        self.args = args
        rospy.loginfo("============Args============")
        rospy.loginfo(args)

        # initialize openai api key
        openai.api_key = os.getenv("OPENAI_API_KEY")
        if openai.api_key is None:
            rospy.logerr("OPENAI_API_KEY is not set")
            return
        rospy.loginfo("============OpenAI API Key============")
        rospy.loginfo(openai.api_key)

        openai_client = openai.OpenAI()

        # check if task_chat_folder exists, if not create it
        if not os.path.exists(self.args.task_chat_folder):
            os.makedirs(self.args.task_chat_folder)
            rospy.logwarn("Created task_chat_folder: {}".format(self.args.task_chat_folder))
        else:
            rospy.loginfo("============task_chat_folder============")
            rospy.loginfo(self.args.task_chat_folder)

        rospy.logwarn("============Start============")

        messages = {}
        full_messages = self.read_messages()
        rospy.loginfo("{} messages, {} full messages".format(len(messages), len(full_messages)))

        while not rospy.is_shutdown():
            func = input("Enter function: ")
            if func == "q":
                rospy.logwarn("============Quit============")
                break
            elif func == "c":
                new_full_messages = self.read_messages()
                if len(new_full_messages) > len(full_messages):
                    rospy.logwarn("{} messages are added".format(len(new_full_messages) - len(full_messages)))
                    full_messages = new_full_messages
                elif len(new_full_messages) < len(full_messages):
                    rospy.logwarn("{} messages are deleted".format(len(full_messages) - len(new_full_messages)))
                    full_messages = new_full_messages
                else:
                    rospy.logwarn("No new messages")
                    full_messages = new_full_messages
                messages = [full_messages[0], full_messages[-2], full_messages[-1]]
                rospy.loginfo("{} messages, {} full messages".format(len(messages), len(full_messages)))
                self.print_messages(messages)
            elif func == "s":
                if len(messages) < 3:
                    rospy.logerr("{} messages are not enough".format(len(messages)))
                    continue
                if messages[0]["role"] != "user":
                    rospy.logerr("First message is not user")
                    continue
                if messages[1]["role"] != "assistant":
                    rospy.logerr("Second message is not assistant")
                    continue
                if messages[2]["role"] != "user":
                    rospy.logerr("Third message is not user")
                    continue
                rospy.logwarn("============Send============")
                try:
                    response = openai_client.chat.completions.create(
                        model=self.args.model_name, messages=messages, temperature=self.args.temperature
                    )
                    self.write_response(str(response), len(full_messages) / 2)
                    self.write_assistant_message(str(response.choices[0].message.content), len(full_messages) / 2)
                except Exception as e:
                    rospy.logerr(e)
                    continue

    def read_mesaage(self, role, file_name):
        message = {}
        message["role"] = role
        message["content"] = ""

        if not os.path.exists(file_name):
            open(file_name, "w").close()

        with open(file_name, "r") as f:
            for line in f:
                message["content"] += line

        return message

    def read_messages(self):
        messages = []

        for i in range(self.args.dialogue_num):
            role = "user"
            file_name = "{}/{}_0_{}.txt".format(self.args.task_chat_folder, i, role)
            message_user = self.read_mesaage(role, file_name)
            if message_user["content"] != "":
                messages.append(message_user)

            role = "assistant"
            file_name = "{}/{}_1_{}.txt".format(self.args.task_chat_folder, i, role)
            message_assistant = self.read_mesaage(role, file_name)
            if message_assistant["content"] != "":
                messages.append(message_assistant)

            if message_user["content"] == "" or message_assistant["content"] == "":
                break

        return messages

    def write_response(self, response, i):
        file_name = "{}/{}_2_response.txt".format(self.args.task_chat_folder, int(i))

        with open(file_name, "w") as f:
            f.write(response)

    def write_assistant_message(self, text, i):
        file_name = "{}/{}_1_assistant.txt".format(self.args.task_chat_folder, int(i))

        with open(file_name, "w") as f:
            f.write(text)

    def print_messages(self, messages):
        rospy.logwarn("============Messages============")
        for message in messages:
            rospy.loginfo(f"{message['role']}: {message['content'][:500]}")


if __name__ == "__main__":
    rospy.init_node("behavior_tree_generation_multi_dialogue")
    args = Args(
        {
            "task_chat_folder": rospy.get_param("~task_chat_folder", ""),
            "dialogue_num": rospy.get_param("~dialogue_num", 20),
            "model_name": rospy.get_param("~model_name", "gpt-3.5-turbo"),
            "temperature": rospy.get_param("~temperature", 0.0),
        }
    )
    multi_dialogue = MultiDialogue(args)
