import roslaunch

def launch_file_and_args(cli_args):
  roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
  roslaunch_args = cli_args[2:]

  return (roslaunch_file, roslaunch_args)


def get_launcher(launch_files: list) -> roslaunch.parent.ROSLaunchParent:
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files, verbose=True)

  return parent

def start_communication_simulator(robots:list, comm_range:float):
  pkg = "simulation"
  executable = "communication_simulator.py"
  node = roslaunch.core.Node(pkg, executable,
    args=f"{comm_range} { ' '.join(robots) }")

  launch = roslaunch.scriptapi.ROSLaunch()
  launch.start()
  process = launch.launch(node)
  return process