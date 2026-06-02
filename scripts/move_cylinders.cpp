#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

static void write_cylinder_states(
    const std::vector<std::pair<std::string,
        std::tuple<double,double,double,double>>>& states)
{
    std::ofstream tmp("/tmp/cylinder_states.tmp");
    if (!tmp) return;
    tmp << std::fixed;
    tmp.precision(4);
    for (const auto& [name, s] : states) {
        auto [x, y, vx, vy] = s;
        tmp << name << " " << x << " " << y << " " << vx << " " << vy << "\n";
    }
    tmp.close();
    std::rename("/tmp/cylinder_states.tmp", "/tmp/cylinder_states");
}

namespace {

struct Cylinder {
  std::string name;
  std::string axis;
  double perp;
  double amplitude;
  double center;
  double period;
  double phase;
};

struct Config {
  double z;
  std::vector<Cylinder> cylinders;
};

std::string slurp(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    std::cerr << "Cannot open " << path << "\n";
    std::exit(1);
  }
  std::stringstream ss; ss << f.rdbuf(); return ss.str();
}

double parseNumberAfter(const std::string& s, size_t pos) {
  while (pos < s.size() && (std::isspace((unsigned char)s[pos]) || s[pos] == ':')) ++pos;
  size_t end = pos;
  while (end < s.size() && (std::isdigit((unsigned char)s[end]) ||
                            s[end] == '.' || s[end] == '-' || s[end] == '+' ||
                            s[end] == 'e' || s[end] == 'E')) ++end;
  return std::stod(s.substr(pos, end - pos));
}

std::string parseStringAfter(const std::string& s, size_t pos) {
  size_t open = s.find('"', pos);
  size_t close = s.find('"', open + 1);
  return s.substr(open + 1, close - open - 1);
}

double findFieldNumber(const std::string& obj, const std::string& key) {
  size_t k = obj.find('"' + key + '"');
  if (k == std::string::npos) {
    std::cerr << "Missing key: " << key << "\n"; std::exit(1);
  }
  return parseNumberAfter(obj, k + key.size() + 2);
}

std::string findFieldString(const std::string& obj, const std::string& key) {
  size_t k = obj.find('"' + key + '"');
  if (k == std::string::npos) {
    std::cerr << "Missing key: " << key << "\n"; std::exit(1);
  }
  return parseStringAfter(obj, k + key.size() + 2);
}

Config loadConfig(const std::string& path) {
  std::string text = slurp(path);
  Config cfg;
  cfg.z = findFieldNumber(text, "z");
  size_t arr = text.find("\"cylinders\"");
  if (arr == std::string::npos) { std::cerr << "No 'cylinders'\n"; std::exit(1); }
  size_t lb = text.find('[', arr);
  size_t rb = text.find(']', lb);
  std::string body = text.substr(lb + 1, rb - lb - 1);

  size_t pos = 0;
  while (pos < body.size()) {
    size_t s = body.find('{', pos);
    if (s == std::string::npos) break;
    int depth = 1; size_t e = s + 1;
    while (e < body.size() && depth > 0) {
      if (body[e] == '{') ++depth;
      else if (body[e] == '}') --depth;
      ++e;
    }
    std::string obj = body.substr(s, e - s);
    Cylinder c;
    c.name = findFieldString(obj, "name");
    c.axis = findFieldString(obj, "axis");
    c.perp = findFieldNumber(obj, "perp");
    c.amplitude = findFieldNumber(obj, "amplitude");
    c.center = findFieldNumber(obj, "center");
    c.period = findFieldNumber(obj, "period");
    c.phase = findFieldNumber(obj, "phase");
    cfg.cylinders.push_back(std::move(c));
    pos = e;
  }
  return cfg;
}

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

struct RobotState {
  std::mutex mutex;
  bool known = false;
  double x = 0.0, y = 0.0;
};
RobotState g_robot;

constexpr double kRobotSafety = 0.30;
constexpr double kGoalX = 1.8;
constexpr double kGoalY = 0.5;
constexpr double kGoalSafety = 0.30;
constexpr double kMaxStep = 0.10;
constexpr const char* kRobotName = "burger";

}

int main(int argc, char** argv) {
  std::string config_path;
  std::string world = "default";
  double rate_hz = 10.0;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--world" && i + 1 < argc) world = argv[++i];
    else if (a == "--rate" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
    else if (config_path.empty()) config_path = a;
    else { std::cerr << "Unknown arg: " << a << "\n"; return 1; }
  }
  if (config_path.empty()) {
    std::cerr << "Usage: move_cylinders <config.json> [--world NAME] [--rate HZ]\n";
    return 1;
  }

  Config cfg = loadConfig(config_path);
  if (cfg.cylinders.empty()) {
    std::cerr << "No cylinders in config — exiting.\n";
    return 0;
  }

  gz::transport::Node node;
  std::string svc = "/world/" + world + "/set_pose";

  std::cerr << "Driving " << cfg.cylinders.size()
            << " cylinder(s) at " << rate_hz << " Hz on world '" << world
            << "' (service " << svc << ")\n";

  std::string pose_topic = "/world/" + world + "/dynamic_pose/info";
  std::function<void(const gz::msgs::Pose_V&)> pose_cb =
      [](const gz::msgs::Pose_V& msg) {
        for (int i = 0; i < msg.pose_size(); ++i) {
          const auto& p = msg.pose(i);
          if (p.name() == kRobotName) {
            std::lock_guard<std::mutex> lk(g_robot.mutex);
            g_robot.x = p.position().x();
            g_robot.y = p.position().y();
            g_robot.known = true;
            return;
          }
        }
      };
  node.Subscribe<gz::msgs::Pose_V>(pose_topic, pose_cb);
  std::cerr << "Subscribed to robot pose on " << pose_topic << "\n";

  std::unordered_map<std::string, std::pair<double, double>> last_pos;
  std::unordered_map<std::string, double> eff_t;

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  auto t0 = std::chrono::steady_clock::now();
  auto next_tick = t0;
  while (!g_stop) {
    double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    bool robot_known;
    double rx, ry;
    {
      std::lock_guard<std::mutex> lk(g_robot.mutex);
      robot_known = g_robot.known;
      rx = g_robot.x; ry = g_robot.y;
    }

    std::vector<std::pair<std::string, std::tuple<double,double,double,double>>> states;

    for (const auto& c : cfg.cylinders) {
      if (eff_t.find(c.name) == eff_t.end()) eff_t[c.name] = t;
      double et = eff_t[c.name];

      double desired_axis =
          c.center + c.amplitude * std::sin(2.0 * M_PI * et / c.period + c.phase);
      double perp_coord = c.perp;

      double current_axis;
      auto it = last_pos.find(c.name);
      if (it == last_pos.end()) {
        current_axis = c.center + c.amplitude * std::sin(c.phase);
      } else {
        current_axis = (c.axis == "x") ? it->second.first : it->second.second;
      }

      double new_axis = desired_axis;
      bool overlapped_robot = false;
      auto apply_exclusion = [&](double e_x, double e_y, double safety,
                                 bool freeze_on_overlap,
                                 bool* overlap_flag = nullptr) {
        double e_axis = (c.axis == "x") ? e_x : e_y;
        double e_perp = (c.axis == "x") ? e_y : e_x;
        double perp_diff = perp_coord - e_perp;
        double perp_dist_sq = perp_diff * perp_diff;
        if (perp_dist_sq >= safety * safety) return;
        double h = std::sqrt(safety * safety - perp_dist_sq);
        double lo = e_axis - h;
        double hi = e_axis + h;
        if (current_axis <= lo) {
          new_axis = std::min(new_axis, lo);
        } else if (current_axis >= hi) {
          new_axis = std::max(new_axis, hi);
        } else {
          if (overlap_flag) *overlap_flag = true;
          if (freeze_on_overlap) {
            new_axis = current_axis;
          } else {
            new_axis = (current_axis < e_axis) ? lo : hi;
          }
        }
      };
      if (robot_known) {
        apply_exclusion(rx, ry, kRobotSafety, true, &overlapped_robot);
      }
      apply_exclusion(kGoalX, kGoalY, kGoalSafety, false);


      double step = new_axis - current_axis;
      if (std::abs(step) > kMaxStep) {
        new_axis = current_axis + (step > 0 ? kMaxStep : -kMaxStep);
      }

      double x, y;
      if (c.axis == "x") { x = new_axis; y = perp_coord; }
      else               { x = perp_coord; y = new_axis; }
      last_pos[c.name] = {x, y};

      if (!overlapped_robot) eff_t[c.name] += 1.0 / rate_hz;

      double vel_axis = overlapped_robot ? 0.0 :
          c.amplitude * (2.0 * M_PI / c.period) *
          std::cos(2.0 * M_PI * et / c.period + c.phase);
      double vx = (c.axis == "x") ? vel_axis : 0.0;
      double vy = (c.axis == "y") ? vel_axis : 0.0;
      states.push_back({c.name, {x, y, vx, vy}});

      gz::msgs::Pose req;
      req.set_name(c.name);
      req.mutable_position()->set_x(x);
      req.mutable_position()->set_y(y);
      req.mutable_position()->set_z(cfg.z);
      req.mutable_orientation()->set_w(1.0);

      gz::msgs::Boolean rep;
      bool ok = false;
      node.Request(svc, req, 100, rep, ok);
      if (!ok || !rep.data()) {
        std::cerr << "set_pose " << c.name << " failed at t=" << t << "\n";
      }
    }

    write_cylinder_states(states);

    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    auto now = std::chrono::steady_clock::now();
    if (next_tick > now) std::this_thread::sleep_until(next_tick);
    else next_tick = now;
  }
  std::cerr << "move_cylinders: shutting down\n";
  return 0;
}
