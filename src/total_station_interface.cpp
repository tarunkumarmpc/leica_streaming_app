/**
 * \file total_station_interface.cpp
 * \author Andreas Ziegler
 * \date 08.06.2018
 * \brief Implementation of the total station interface base class
 */

#include <iostream>
#include <vector>
#include <boost/lexical_cast.hpp>
#include "leica_streaming_app/total_station_interface.h"

// Constructor for TSInterface
TSInterface::TSInterface(
    std::function<void(const double, const double, const double)> locationCallback,
    const rclcpp::NodeOptions& options)
  : Node("ts_interface", options),  // Pass options to the base class constructor
    io_context_(std::make_unique<boost::asio::io_service>()),
    timer_(*io_context_, boost::posix_time::seconds(2)),
    timerStartedFlag_(false),
    searchingPrismFlag_(false),
    externalPositionReceivedFlag_(false),
    prismPosition_(3),
    tsState_(TSState::on),
    locationCallback_(locationCallback) {}



// Start the timer
void TSInterface::startTimer() {
  std::cout << "Start timer" << std::endl;
  timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(2));
  timer_.async_wait(std::bind(&TSInterface::timerHandler, this));
}

// Start command for the total station
void TSInterface::start() {
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '1', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::on;
}

// End command for the total station
void TSInterface::end() {
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '2', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::off;
}

// Set the position of the prism
void TSInterface::setPrismPosition(double x, double y, double z) {
  {
    std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
    prismPosition_ = {x, y, z}; // Use initializer list for clarity
    externalPositionReceivedFlag_ = true; // Set flag indicating position received
  }
}

// Timer handler to check for messages and manage prism search
void TSInterface::timerHandler() {
  if (tsState_ == TSState::on) {
    std::lock_guard<std::mutex> guard1(messageReceivedMutex_);
    std::lock_guard<std::mutex> guard2(searchingPrismMutex_);

    if (!messagesReceivedFlag_ && !searchingPrismFlag_) {
      std::cout << "Prism lost!" << std::endl;

      std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
      if (externalPositionReceivedFlag_) {
        turnTelescope(); // Turn telescope if a recent external position was received
      }

      searchPrism(); // Start searching for the prism
    }

    messagesReceivedFlag_ = false; // Reset message received flag

    // Reset external position flag
    externalPositionReceivedFlag_ = false;
  }

  // Restart timer with a shorter interval for responsiveness
  timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(800));
  timer_.async_wait(std::bind(&TSInterface::timerHandler, this));
}

// Command to search for the prism
void TSInterface::searchPrism(void) {
  searchingPrismFlag_ = true;
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '6', ':', '1', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  
  std::cout << "Search prism" << std::endl;
}

// Command to turn the telescope to the prism's position
void TSInterface::turnTelescope(void) {
  searchingPrismFlag_ = true;
  
  // Create command to turn telescope with prism coordinates
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '7', ':', '1'};
  
  // Append prism coordinates to command
  for (const auto& position : prismPosition_) {
    std::string posStr = boost::lexical_cast<std::string>(position);
    command.insert(command.end(), posStr.begin(), posStr.end());
  }
  
  command.emplace_back(0x0d/*CR*/);
  command.emplace_back(0x0a/*LF*/);

  write(command); // Send command to turn telescope
}
