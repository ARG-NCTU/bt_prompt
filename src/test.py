from bt_generator import BTGenerator

gen = BTGenerator(
    path="/home/argrobotx/robotx-2022/catkin_ws/src/bt_prompt/config/gpt3_temperature_1/L/visual_servoing",
    sub_tree_dir_path="/home/argrobotx/robotx-2022/catkin_ws/src/bt_prompt/config/subtree",)
gen.read_prompt()
response = gen.generate()
gen.write_response(response)
gen.write_raw(response)
gen.write_tree(gen.generate_tree(response))