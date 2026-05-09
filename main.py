from flask import Flask, request, jsonify
from flask_cors import CORS
from typing import Optional, List, Dict, Any
import requests
import json
import os
import re
import math
import yaml
import psutil
from langchain.agents import initialize_agent, AgentType, Tool
from langchain.memory import ConversationBufferMemory
from langchain.llms.base import LLM
import rospy
import rosnode
import subprocess
import time
from collections import deque

script_dir = os.path.dirname(os.path.abspath(__file__))

class ProcessManager:
    def __init__(self):
        self.graph = {}
        self.launch_files = {}

        self.add_state("IDLE")
        self.add_state("INIT")
        self.add_state("EXPM")
        self.add_state("EXRL")
        self.add_state("TASK")
        self.curr_state = "IDLE"

        self.add_transition("IDLE", "INIT")
        self.add_transition("INIT", "EXPM")
        self.add_transition("INIT", "EXRL")
        self.add_transition("INIT", "TASK")
        self.add_transition("EXPM", "TASK")
        self.add_transition("EXRL", "TASK")

        with open(os.path.join(script_dir, "ui/database/config.yaml"), 'r') as file:
            self.config = yaml.safe_load(file)

    def add_state(self, state):
        if state not in self.graph:
            self.graph[state] = []

    def add_transition(self, from_state, to_state):
        if from_state in self.graph:
            self.graph[from_state].append(to_state)
        else:
            print(f"State {from_state} does not exist. Please add it first.")

    def find_shortest_path(self, start_state, end_state):
        queue = deque([(start_state, [start_state])])
        visited = set()
        while queue:
            current_state, path = queue.popleft()
            if current_state == end_state:
                return path[1:]
            visited.add(current_state)
            for neighbor in self.graph.get(current_state, []):
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
                    visited.add(neighbor)
        return None

    def run_state_files(self, next_state):
        path = self.find_shortest_path(self.curr_state, next_state)
        for state in path:
            if state=="INIT":
                self.run_launch_file({
                    "sess": "world",
                    "pack": "custom_world",
                    "file": "custom_world.launch",
                    "args": f"num_bots:={self.config['num_bots']}",
                    "node": f"/bot{self.config['num_bots']}/spawn_bot{self.config['num_bots']}"
                })
                for i in range(self.config["num_bots"]):
                    self.run_launch_file({
                        "sess": "yolo8",
                        "pack": "ultralytics_ros",
                        "file": "tracker.launch",
                        "args": f"bot_num:={i+1}",
                        "node": f"/tracker_node{i+1}"
                    })
                    self.run_launch_file({
                        "sess": "deeps",
                        "pack": "deepsort_ros",
                        "file": "sort_deep.launch",
                        "args": f"bot_num:={i+1}",
                        "node": f"/deepsort_node{i+1}"
                    })
                self.run_launch_file({
                    "sess": "datab",
                    "pack": "data_manager",
                    "file": "data_handler.launch",
                    "args": "",
                    "node": "/whole_handler"
                })

            if state=="EXPM":
                self.run_launch_file({
                    "sess": "mapge",
                    "pack": "custom_world",
                    "file": "map_merger.launch",
                    "args": f"num_bots:={self.config['num_bots']}",
                    "node": "/map_merge"
                })
                self.run_launch_file({
                    "sess": "front",
                    "pack": "explore_lite",
                    "file": "explore.launch",
                    "args": f"num_bots:={self.config['num_bots']}",
                    "node": f"/bot{self.config['num_bots']}/explore"
                })
                self.run_launch_file({
                    "sess": "teblp",
                    "pack": "teb_local_planner",
                    "file": "teb_test.launch",
                    "args": f"num_bots:={self.config['num_bots']}",
                    "node": f"/bot{self.config['num_bots']}/move_base"
                })
                self.run_launch_file({
                    "sess": "thres",
                    "pack": "remain_package",
                    "file": "frontiers_check.launch",
                    "args": "",
                    "node": "/map_explored"
                })

            if state=="TASK":
                self.run_launch_file({
                    "sess": "mapge",
                    "pack": "custom_world",
                    "file": "map_merger.launch",
                    "args": f"num_bots:={self.config['num_bots']}",
                    "node": "/map_merge"
                })
                self.run_launch_file({
                    "sess": "teblp",
                    "pack": "teb_local_planner",
                    "file": "teb_test.launch",
                    "args": f"num_bots:={self.config['num_bots']}",
                    "node": f"/bot{self.config['num_bots']}/move_base"
                })
            
            self.curr_state = state

    def run_launch_file(self, launch_file):
        session_name = launch_file['sess']
        source_command = "source devel/setup.sh"
        roslaunch_command = f"roslaunch {launch_file['pack']} {launch_file['file']} {launch_file['args']}"
        print(f"Running '{roslaunch_command}'")
        subprocess.Popen([
            "gnome-terminal", "--tab", "--", "bash", "-c",
            f"echo 'Session: {session_name}'; {source_command} && {roslaunch_command}; exec bash"
        ])

        if self.wait_for_node(launch_file["node"]):
            print(f"Node {launch_file['node']} started successfully.")
        else:
            print(f"Failed to start node {launch_file['node']}. Restarting...")
    
    def kill_running_session(self, session_name):
        for proc in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
            if proc.info["cmdline"] and session_name in " ".join(proc.info["cmdline"]):
                print(f"Found process with session name '{session_name}': PID {proc.info['pid']}")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except psutil.TimeoutExpired:
                    print(f"Force killing process {proc.pid}.")
                    proc.kill() 
                print(f"Process with PID {proc.info['pid']} has been killed.")
                return True
        print(f"No process found with session name '{session_name}'.")
        return False

    def wait_for_node(self, node_name):
        if node_name is None: 
            return True
        while not rospy.is_shutdown():
            if self.is_node_running(node_name):
                return True
            time.sleep(1)
        return False
    
    def is_node_running(self, node_name):
        try:
            nodes = rosnode.get_node_names()
            return node_name in nodes
        except Exception as e:
            print(f"Error checking nodes: {e}")
            return False
        
class CustomizedLLM(LLM):
    api_base_url: str = "https://llamatool.us.gaianet.network/v1"
    model_name: str = "llama"
    api_key: str = "GAIA"
    def _call(self, prompt: str, stop: Optional[List[str]] = None) -> str:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        payload = {
            "model": self.model_name,
            "prompt": prompt,
            "max_tokens": 1000  
        }
        if stop:
            payload["stop"] = stop
        response = requests.post(f"{self.api_base_url}/completions", json=payload, headers=headers)
        if response.status_code == 200:
            data = response.json()
            return data["choices"][0]["text"].strip() 
        else:
            raise Exception(f"API request failed with status code {response.status_code}: {response.text}")
    @property
    def _identifying_params(self) -> Dict[str, Any]:
        return {"model_name": self.model_name}
    @property
    def _llm_type(self) -> str:
        return "custom_llama"
    
class ToolManager(ProcessManager):
    def exploration_tool(self):
        def exploration_function(query: str) -> str:
            print("Inside exploration_function...")
            self.run_state_files("EXPM")
            self.kill_running_session("mapge")
            self.kill_running_session("front")
            self.kill_running_session("teblp")
            response = "Exploration Completed Succesfully"
            return response
        response_tool = Tool(
            name="ExplorationTool",
            func=exploration_function,
            description="A tool for running Exploration. Like if you want swarm to explore the environment then this tool will get used."
        )
        return response_tool
    
    def goto_object_tool(self):
        def extract_object_name(query: str) -> str:
            try:
                match = re.search(r"\b(?:find|go to)\b\s*(.+)|^(\w+)$", query.lower())
                if match:
                    object_name = match.group(1) or match.group(2)
                    print(f"Debug: Extracted object name: {object_name}")
                    return object_name.strip()
                else:
                    raise ValueError("Could not understand the query format.")
            except Exception as e:
                return str(e)
        def extract_object_location(object_name):
            objects_file = os.path.join(script_dir, "ui/database/objects.json")
            with open(objects_file, 'r') as file:
                objects = json.load(file)
            for object in objects:
                if object["name"] == object_name:
                    return object["loc_x"], object["loc_y"]
        def extract_closest_robot(loc_x, loc_y):
            robots_file = os.path.join(script_dir, "ui/database/robots.json")
            with open(robots_file, 'r') as file:
                robots = json.load(file) 
            closest_robot = None
            min_distance = float('inf')
            for robot in robots:
                robot_x = robot["loc_x"]
                robot_y = robot["loc_y"]
                distance = math.sqrt((loc_x - robot_x) ** 2 + (loc_y - robot_y) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    closest_robot = robot["name"]
            return closest_robot
        def goto_object_function(query: str) -> str:
            print("Inside goto_function_function...")
            self.run_state_files("TASK")
            object_name = extract_object_name(query)
            loc_x, loc_y = extract_object_location(object_name)
            closest_robot = extract_closest_robot(loc_x, loc_y)
            self.run_launch_file({
                "sess": "taskf",
                "pack": "swarm_tasks",
                "file": "goto_object.launch",
                "args": f"bot:={closest_robot} loc_x:={loc_x} loc_y:={loc_y}",
                "node": "/goal_publisher"
            })
            response = "Task Completed Succesfully"
            return response
        response_tool = Tool(
            name="GotoObjectTool",
            func=goto_object_function,
            description="A tool for bot to move to some object, or find this object."
        )
        return response_tool

custom_llm = CustomizedLLM()
tool_manager = ToolManager()
tools = [
    tool_manager.exploration_tool(),
    tool_manager.goto_object_tool()
]

memory = ConversationBufferMemory(memory_key="chat_history")
memory.chat_memory.add_user_message("Hello")
memory.chat_memory.add_ai_message("Hi, how can i help you?")

agent = initialize_agent(
    tools,
    custom_llm,
    agent=AgentType.CONVERSATIONAL_REACT_DESCRIPTION,
    verbose=True,
    memory= memory,
    handle_parsing_errors=True
)
print("yup")

app = Flask(__name__)
CORS(app)

@app.route('/assign', methods=['POST'])
def agent_endpoint():
    data = request.get_json()
    user_input = data['user_input']
    if not user_input:
        return jsonify({"error": "No input provided"}), 400
    #try:
    response = agent.invoke(user_input)
    return jsonify({"success": True, "response": response['output'].replace("<|eot_id|>", "").strip()})
    #except Exception as e:
    #    return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True)
