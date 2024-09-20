
#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <functional>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Possible states of the total station.
 */
enum class TSState { off, on };

class TSInterface : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the TSInterface class.
   * 
   * @param locationCallback A callback function to handle location updates.
   * @param options Node options for ROS 2.
   */
  explicit TSInterface(
      std::function<void(const double, const double, const double)> locationCallback,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  /**
   * @brief Sends the start command to the total station.
   */
  void start(void);

  /**
   * @brief Sends the end command to the total station.
   */
  void end(void);

  /**
   * @brief Set the position of the prism to track externally.
   *
   * @param x X coordinate
   * @param y Y coordinate
   * @param z Z coordinate
   */
  void setPrismPosition(double x, double y, double z);

 protected:
  /**
   * @brief Starts the timer to detect if no messages are received anymore.
   */
  void startTimer();

  /**
   * @brief Sends a command to the total station.
   *
   * @param command A vector of characters containing the command.
   */
  virtual void write(std::vector<char> str) = 0;

  /**
   * @brief Callback method for the timer.
   * 
   * This method starts searching for the prism if no message was received since last time,
   * and restarts the timer at the end.
   */
  void timerHandler();

  /**
   * @brief Starts the prism search on the total station.
   */
  void searchPrism(void);

  /**
   * @brief Turns the total station's telescope to the position received externally.
   */
  void turnTelescope(void);

  TSState tsState_;                                       /**< State of the total station */
  
  std::vector<double> prismPosition_;                     /**< Position of the prism given by Rovio or Vicon */

  std::unique_ptr<boost::asio::io_service> io_context_;   /**< IO context object */

  boost::asio::streambuf readData_;                       /**< Stream buffer for incoming messages */

  bool timerStartedFlag_;                                 /**< Flag indicating if the timer has started */
  
  boost::asio::deadline_timer timer_;                     /**< Deadline timer for message detection */

  bool messagesReceivedFlag_;                             /**< Flag indicating if a message was received */

  std::mutex messageReceivedMutex_;                       /**< Mutex for synchronizing message received flag */

  bool searchingPrismFlag_;                               /**< Flag indicating if the total station is searching for a prism */

  std::mutex searchingPrismMutex_;                        /**< Mutex for synchronizing searching prism flag */

  bool externalPositionReceivedFlag_;                     /**< Flag indicating if a recent position was received externally */

  std::mutex externalPositionReceivedMutex_;              /**< Mutex for synchronizing external position flag */

  std::function<void(const double, const double, const double)> locationCallback_; /**< Callback function for location updates */
};

