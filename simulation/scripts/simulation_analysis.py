#!/usr/bin/env python3

from argparse import ArgumentParser
import os
import json

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

def is_number(x):
  if type(x) == int:
    return True
  
  if type(x) == float:
    return True
  
  return False

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
    self._mean_position_iae = []
    self._mean_heading_iae = []
    self._status_register = ResultStatus.get_status_dict()

  def safe_append(self, vector: list, value):
    if is_number(value):
      vector.append(value)

  def read_robot_statistics(self, statistics: dict):
    self._area_coverage_time.append(statistics["area_coverage_time"])
    self.safe_append(self._mean_position_iae, statistics["mean_position_iae"])
    self.safe_append(self._mean_heading_iae, statistics["mean_heading_iae"])

  def process_info(self, info: dict):
    status = ResultStatus.translate(info[self._status])
    if status == ResultStatus.success:
      self._status_register[ResultStatus.success] += 1
      statistics = info["statistics"]
      for _, robot_stats in statistics.items():
        self.read_robot_statistics(robot_stats)
    else:
      status = ResultStatus.translate(info[self._reason])
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
    self.fill_statistic("mean_position_iae", self._mean_position_iae, statistics)
    self.fill_statistic("mean_heading_iae", self._mean_heading_iae, statistics)


    result = {}
    result["status"] = status
    result["success_statistics"] = statistics

    return result

def Folder(path: str) -> str:
  
  if not os.path.exists(path):
    raise ValueError
  
  if not os.path.isdir(path):
    raise TypeError
  
  return path

def get_parser() -> ArgumentParser:
  parser = ArgumentParser()

  parser.add_argument("results_folder", type=Folder, help="folder \
    with the simulations")

  return parser

def results(results_folder: Folder):
  for path in os.listdir(results_folder):
    full_path = os.path.join(results_folder, path)
    try:
      yield Folder(full_path)
    except:
      print(f"{full_path} is not a folder")

def get_result_info(result_folder: Folder) -> dict:
  info_file = os.path.join(result_folder, "info.json")
  with open(info_file, 'r') as f:
    return json.load(f)

def process_results(results):
  statistics = Statistics()
  for result in results:
    result_info = get_result_info(result)
    statistics.process_info(result_info["simulation_result"])
  
  return statistics

if __name__ == "__main__":
  parser = get_parser()
  args = parser.parse_args()


  statistics = process_results(results(args.results_folder))

  print(statistics.get())
