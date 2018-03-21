#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <list>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define TWIDDLE 0

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Initialize the PID parameter
static std::array<double, 3> k = { {0.15, 3, 0.00012} };

// The target speed
static const double targetSpeed = 50.0;
static const bool reduceSpeedOnGreatCte = true;

#if TWIDDLE
constexpr unsigned lengthOfTwiddleTest = 3500;
static unsigned dataNum = 0;
static std::array<double, 3> dk = { {k[0]/2, k[1]/2, k[2]/2} };
static bool dirChanged = true;
static double overallError = 0;
static double prevError = 1e+12;
static double bestError = prevError;
static unsigned twiddleIdx = 2;
static std::list<double> savedResults;

void saveSettingsAndResults(double kp, double kd, double ki, double err) {
  savedResults.push_back(err);
  double mean = 0;
  for (auto x : savedResults) {
      mean += x;
  }
  mean /= savedResults.size();
  double variance = 0;
  for (auto x : savedResults) {
    const double err = x - mean;
    variance += err*err;
  }
  variance /= savedResults.size();

  std::ofstream resultFile("pid_results.csv", std::ios_base::app);
  if (resultFile.is_open()) {
    resultFile << kp << "\t" << kd << "\t" << ki << "\t"
               << err << "\t" << mean << "\t" << variance << std::endl;
    resultFile.close();
  }
}

// Twiddle the PID parameters to reduce the cross track error
void twiddle(const double currError) {
#if 0
  auto oldK = k;
  auto oldDK = dk;
  auto oldDir = dirChanged;
  if (currError < bestError) {
    bestError = currError;
    // Continue to change the parameter in the same direction as in the last step
    k[twiddleIdx] += dk[twiddleIdx];
  }
  else {
    if (dirChanged) {
      dk[twiddleIdx] /= 3;
      dirChanged = false;
    }
    else {
      dk[twiddleIdx] *= -1;
      dirChanged = true;
      k[twiddleIdx] += dk[twiddleIdx];
    }
    k[twiddleIdx] += dk[twiddleIdx];
  }
  std::printf("twiddle() k:%lf/%lf/%lf dk:%lf/%lf/%lf dir:%u"
              "-> k:%lf/%lf/%lf dk:%lf/%lf/%lf dir:%u\n",
              oldK[0], oldK[1], oldK[2], oldDK[0], oldDK[1], oldDK[2], oldDir,
              k[0], k[1], k[2], dk[0], dk[1], dk[2], dirChanged);
#endif
}
#endif

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID steerPid(-1.0, 1.0);
  steerPid.Init(k[0], k[1], k[2]);

  PID throttlePid(-4.0, 4.0);
  throttlePid.Init(0.2, 0, 0.0);

  h.onMessage([&steerPid, &throttlePid](uWS::WebSocket<uWS::SERVER> ws,
                     char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
//          std::cout << "CTE: " << cte
//                    << " Speed: " << speed
//                    << " Angle: " << angle << std::endl;

          static double maxCte = 0.0;
          if (fabs(cte) > maxCte)
          {
            maxCte = fabs(cte);
            std::cout << "max cte:" << maxCte << &std::endl;
          }

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          double steer_value = steerPid.CalculateControlValue(cte, angle, speed);

          // cte goes up to
          double currentTargetSpeed = targetSpeed;
          if (reduceSpeedOnGreatCte) {
            currentTargetSpeed *= (1 - 0.25*cte);
            if (0 > currentTargetSpeed) {
              currentTargetSpeed = 0;
            }
          }
          double speedErr = speed - currentTargetSpeed;
          double throttle_value = throttlePid.CalculateControlValue(speedErr, angle, speed);

#if TWIDDLE
          overallError += cte*cte;

          ++dataNum;
          if (0 == (dataNum % lengthOfTwiddleTest)) {
            // After about one round
            std::cout << "Track over: error=" << overallError << std::endl;
            saveSettingsAndResults(k[0], k[1], k[2], overallError);
            twiddle(overallError);
            savedResults.clear();
            // Reset the error
            prevError = overallError;
            overallError = 0.0;
            steerPid.Init(k[0], k[1], k[2]);
            throttlePid.Reset();
            std::string msg("42[\"reset\",{}]");
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else
#endif
          {

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res,
                     uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws,
                         int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
