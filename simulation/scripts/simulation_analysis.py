#!/usr/bin/env python3

from utils import FullPath, Folder, draw_environment, results_gen, is_number

from argparse import ArgumentParser
import os
import json
import matplotlib.pyplot as plt
import seaborn as sns

from scipy.stats import shapiro
from scipy.stats import norm
import numpy as np
import pickle

class ResultStatus:
  success = "success"
  tle = "tle"
  divergent = "divergent"

  _success = "success"
  _tle = "time limit exceeded"
  _divergent = "robot position error too large"

  def translate(status: str):
    if status == ResultStatus._success:
      return ResultStatus.success
    
    if status == ResultStatus._tle:
      return ResultStatus.tle
    
    if status == ResultStatus._divergent:
      return ResultStatus.divergent
  
  def get_status_dict() -> dict:
    status = {}
    status["success"] = 0
    status["tle"] = 0
    status["divergent"] = 0

    return status

def get_avg_and_stddev(data: list):
  n = len(data)
  avg = sum(data) / n

  s = 0
  for point in data:
    s += (point - avg)**2

  stddev = ( s / (n-1) )**.5 

  return avg, stddev

class Statistics:
  _status = "status"
  _reason = "reason"

  def __init__(self):
    self._area_coverage_time = []
    self._position_iae = []
    self._heading_iae = []
    self._initial_pos = []
    self._status_register = ResultStatus.get_status_dict()

  def safe_append(self, vector: list, value):
    if is_number(value):
      vector.append(value)

  def read_robot_statistics(self, statistics: dict):
    self._area_coverage_time.append(statistics["area_coverage_time"])
    self.safe_append(self._position_iae, statistics["position_iae"])
    self.safe_append(self._heading_iae, statistics["heading_iae"])

  def read_setup_info(self, setup: dict):
    pass
  
  def process_info(self, result_info: dict, robots_setup_info: dict):
    status = ResultStatus.translate(result_info[self._status])
    if status == ResultStatus.success:
      self._status_register[ResultStatus.success] += 1
      statistics = result_info["statistics"]
      # for _, robot_stats in statistics.items():
        # self.read_robot_statistics(robot_stats)
      for robot in robots_setup_info:
        robot_name = robot["name"]
        self._initial_pos.append(tuple(robot["initial_position"]))
        self.read_robot_statistics(statistics[robot_name])
    else:
      status = ResultStatus.translate(result_info[self._reason])
      self._status_register[status] += 1

  def fill_status_count(self, status: str, register: dict, total_simulations) -> None:
    register[status] = {"count": self._status_register[status],
      "percentage": self._status_register[status] / total_simulations}
  
  def fill_statistic(self, statistic_name, data: list, statistic_register: dict) -> None:
    avg, stddev = get_avg_and_stddev(data)
    statistics = {
      "avg": avg,
      "stddev": stddev
    }

    statistic_register[statistic_name] = statistics


  def get(self):
    total_simulations = 0
    for _, n in self._status_register.items():
      total_simulations += n

    status = {}
    self.fill_status_count(ResultStatus.success, status, total_simulations)
    self.fill_status_count(ResultStatus.tle, status, total_simulations)
    self.fill_status_count(ResultStatus.divergent, status, total_simulations)
    
    statistics = {}
    self.fill_statistic("area_coverage_time", self._area_coverage_time, statistics)
    self.fill_statistic("position_iae", self._position_iae, statistics)
    self.fill_statistic("heading_iae", self._heading_iae, statistics)


    result = {}
    result["status"] = status
    result["success_statistics"] = statistics

    return result

def get_parser() -> ArgumentParser:
  parser = ArgumentParser()

  parser.add_argument("results_folder", type=Folder, help="folder \
    with the simulations")
  
  parser.add_argument("experiment_name", type=str)

  return parser

def get_result_info(result_folder: Folder) -> dict:
  info_file = os.path.join(result_folder, "info.json")
  with open(info_file, 'r') as f:
    return json.load(f)

def process_results(results):
  statistics = Statistics()
  for result in results:
    result_info = get_result_info(result)
    statistics.process_info(result_info["simulation_result"],
      result_info["setup"]["robots"])
    # correct_iae_data(result)
  
  return statistics

# def correct_iae_data(result_folder):
#   result_info = get_result_info(result_folder)
#   simulation_result = result_info["simulation_result"]
#   if simulation_result["status"] != ResultStatus.success:
#     return
  
#   statistics = simulation_result["statistics"]
#   for robot_name in statistics:
#     robot_pose_path = os.path.join(result_folder, f"poses-{robot_name}.pickle")
#     with open(robot_pose_path, 'rb') as f:
#       robot_pose = pickle.load(f)
#     n = len(robot_pose.slam)
#     wrong_pos_iae = statistics[robot_name]["mean_position_iae"]
#     del statistics[robot_name]["mean_position_iae"]
#     wrong_heading_iae = statistics[robot_name]["mean_heading_iae"]
#     del statistics[robot_name]["mean_heading_iae"]
#     statistics[robot_name]["position_iae"] = wrong_pos_iae * n
#     statistics[robot_name]["heading_iae"] = wrong_heading_iae * n

#     info_file = os.path.join(result_folder, "info.json")
#     with open(info_file, "w+") as f:
#       json_str = json.dumps(result_info, indent=2)
#       f.write(json_str)

def save_statistics(statistics: dict, filename):
  with open(filename, "w+") as f:
    json_str = json.dumps(statistics, indent=2)
    f.write(json_str)

def plot_success_comparison_rate(results: dict, experiment_name: str,
  full_path: FullPath):
  result_labels = ["Sucesso", "Divergência", "Tempo limite excedido"]
  field = "percentage"
  data = [results[ResultStatus.success][field],\
    results[ResultStatus.divergent][field],\
    results[ResultStatus.tle][field]]

  plt.figure()
  plt.bar(result_labels[0], data[0])
  plt.bar(result_labels[1], data[1])
  plt.bar(result_labels[2], data[2])
  
  plt.xticks(result_labels)

  plt.ylabel("Percentual")
  plt.xlabel("Resultados das simulações")
  plt.title(f"Taxas de sucesso e erro para o cenário com {experiment_name}")

  plt.grid(axis='x')
  
  plt.savefig(full_path.get("success_rate_bar.pdf"))
  # plt.savefig(full_path.get("success_rate_bar.pgf"))

def plot_normal(statistics: dict, data: list):
  _, bins, _ = plt.hist(data, bins='auto', density=True)

  sigma = statistics["stddev"]
  mu = statistics["avg"]
  
  x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
  # plt.plot(x, norm.pdf(x, mu, sigma), linewidth=4)

  # plt.xticks([int(b) for b in bins])

def plot_coverage_time_distribution(statistics: dict, data: list,\
    experiment_name:str, full_path: FullPath):
  f = plt.figure()
  
  plot_normal(statistics, data)

  plt.xlabel("Tempo (s)")
  plt.title(f"Distribuição do tempo de mapeamento {experiment_name}")

  plt.savefig(full_path.get("time-coverage.pdf"))
  # plt.savefig(full_path.get("time-coverage.pgf"))
  plt.close(f)

def plot_position_mean_iae(statistics: dict, data: list,\
    experiment_name: str, full_path: FullPath):

  f = plt.figure()

  plt.hist(data, bins='auto')

  plt.xlabel("Erro (m)")
  plt.title(f"Histograma do IAE médio da posição para o cenário \
    de {experiment_name}")

  plt.savefig(full_path.get("mean-iae-position-histogram.pdf"))
  # plt.savefig(full_path.get("mean-iae-position-histogram.pgf"))

  plt.close(f)

def plot_heading_mean_iae(statistics: dict, data: list,\
    experiment_name: str, full_path: FullPath):

  f = plt.figure()

  plt.hist(data, bins='auto')

  plt.xlabel("Erro (rad)")
  plt.title(f"Histograma do IAE médio da orientação para o cenário \
    de {experiment_name}")

  plt.savefig(full_path.get("mean-iae-heading-histogram.pdf"))
  # plt.savefig(full_path.get("mean-iae-heading-histogram.pgf"))

  plt.close(f)

def plot_initial_positions(initial_positions:list, coverage_time:list,\
    experiment_name:str, full_path: FullPath):
  fig, ax = plt.subplots()
  plt.grid(False)

  draw_environment(ax)

  x = [p[0] for p in initial_positions]
  y = [p[1] for p in initial_positions]

  
  color_map = 'bwr'
  density = plt.scatter(x, y, c=coverage_time, cmap=color_map)
  plt.colorbar(density, cmap=color_map, \
    label="Tempo de mapeamento de 90\% da área (s)")

  plt.legend(bbox_to_anchor=(-0.3, 1.04), loc='upper left')
  plt.title(f"Posições iniciais e tempo de mapeamento {experiment_name}")
  plt.savefig(full_path.get(f"initial_positions_and_time_{experiment_name}.pdf"))
  # plt.savefig(full_path.get(f"initial_positions_and_time_{experiment_name}.pgf"))

def get_csv_header(robots_info: dict):
  header = "experiment"
  for i in range(len(robots_info)):
    header += "; robot_name; initial_position; area_coverage_time; \
      position_iae; heading_iae"
    
  header += '\n'
  
  return header

def get_robot_info_summary(info:dict):
  if info["simulation_result"]["status"] != ResultStatus.success:
    return ""

  setup_info = info["setup"]["robots"]
  result_info = info["simulation_result"]["statistics"]
  row = ""
  for robot in setup_info:
    robot_name = robot["name"]
    result = result_info[robot_name]
    row += f"{robot_name}; {tuple(robot['initial_position'])}; \
      {result['area_coverage_time']:.0f}; {result['position_iae']:.1f}; \
        {result['heading_iae']:.1f};"
    
  return row[:-1]

def infos_to_csv(results, output_file):
  result = next(results)
  first = get_result_info(result)
  header = get_csv_header(first["setup"]["robots"])
  first_row = get_robot_info_summary(first)

  with open(output_file, "w+") as output:
    output.write(header)
    experiment_name = result.split('/')[-1]
    output.write(f"{experiment_name}; {first_row}")
    output.write("\n")
    for result in results:
      info = get_result_info(result)
      row = get_robot_info_summary(info)
      if len(row) > 0:
        experiment_name = result.split('/')[-1]
        output.write(f"{experiment_name}; {row}")
        output.write("\n")
        # print(f"{experiment_name}; {row}")

if __name__ == "__main__":
  parser = get_parser()
  args = parser.parse_args()

  statistics = process_results(results_gen(args.results_folder))

  full_path = FullPath(args.results_folder)

  save_statistics(statistics.get(), full_path.get("statistics.json"))
  print(statistics.get())
  infos_to_csv(results_gen(args.results_folder), full_path.get("info.csv"))
  
'''
  plot_initial_positions(statistics._initial_pos,
    statistics._area_coverage_time, args.experiment_name, full_path)
  
  # plotting
  # # Use the seborn style
  plt.style.use('seaborn-colorblind')
  sns.set_style("whitegrid")

  # # But with fonts from the document body
  plt.rcParams.update({
    "font.family": "serif",  # use serif/main font for text elements
    "font.size": 12,  # use serif/main font for text elements
    "text.usetex": True,     # use inline math for ticks
    "pgf.rcfonts": False     # don't setup fonts from rc parameters
  })


  plot_success_comparison_rate(statistics.get()["status"], args.experiment_name,
    full_path)

  # print(shapiro(statistics._area_coverage_time))
  stats = statistics.get()["success_statistics"]
  filtered = [x for x in statistics._area_coverage_time if x < 700]
  plot_coverage_time_distribution(stats["area_coverage_time"],\
    statistics._area_coverage_time, args.experiment_name, full_path)

  # print(shapiro(statistics._position_iae))
  plot_position_mean_iae(stats["position_iae"],\
    statistics._position_iae, args.experiment_name, full_path)
  
  plot_heading_mean_iae(stats["heading_iae"],\
    statistics._heading_iae, args.experiment_name, full_path)
'''