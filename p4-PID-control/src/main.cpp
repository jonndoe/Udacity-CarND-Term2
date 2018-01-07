#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

int main(int argc, char** argv)
{
	uWS::Hub h;

	PID steering_pid;
	PID throttle_pid;

	// Gain factor for initial steering PID and target speed relative to steering angle
	double steering_gain = 3.75;

	// Initialize the steering PID controller
	steering_pid.Init(0.09, 0.00002, 2.3);

	// Initialize the throttle PID controller
	throttle_pid.Init(0.115, 0.0006, 0.855);


	h.onMessage([&steering_pid, &throttle_pid, &steering_gain, &argv, &argc]\
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
					//double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double steer_value;

					// Coefficients to control speed relative to steering target
					float min_speed = 20.0;
					float add_speed = 30.0;

					// Calculate steering Value
					steering_pid.UpdateError(cte);
					steer_value = steering_pid.Calculate();

					// Set target speed
					double target_speed = add_speed*(1.0-steering_gain*fabs(steer_value)) + min_speed;

					// Calculate speed error
					double speed_error = speed - target_speed;

					// Calculate throttle value
					throttle_pid.UpdateError(speed_error);
					double throttle_value = throttle_pid.Calculate();

					printf("\nSteering coefficients: (%.06f, %.06f, %.06f)", steering_pid.Kp,steering_pid.Ki,steering_pid.Kd );
					printf("\nThrottle coefficients: (%.06f, %.06f, %.06f)\n", throttle_pid.Kp,throttle_pid.Ki,throttle_pid.Kd );

          // DEBUG
					std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle_value;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << std::endl;
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

					if(argc > 1 && strcmp(argv[1], "twiddle") == 0)
					{
						int twiddle_count_threshold = 1000;
						if(steering_pid.update_count_ > twiddle_count_threshold)
							steering_pid.Twiddle(cte);
						if(throttle_pid.update_count_ > twiddle_count_threshold)
							throttle_pid.Twiddle(cte);
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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
