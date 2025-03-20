#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <sys/ioctl.h>
#include <errno.h>
#include <regex>

/*
 * FileDescriptorFlagGuard: RAII guard to temporarily set and restore file descriptor flags.
 * Ensures flags are restored when the guard goes out of scope, keeping writes blocking
 * while allowing temporary non-blocking reads.
 */
class FileDescriptorFlagGuard {
    public:
        FileDescriptorFlagGuard(int fd, int flag_to_set)
            : fd_(fd) {
            // Get current flags
            original_flags_ = fcntl(fd_, F_GETFL, 0);
            if (original_flags_ == -1) {
                std::cerr << "fcntl F_GETFL error: " << errno << std::endl;
                throw std::runtime_error("Failed to get file descriptor flags");
            }
    
            // Set new flags with the specified flag added
            if (fcntl(fd_, F_SETFL, original_flags_ | flag_to_set) == -1) {
                std::cerr << "fcntl F_SETFL error: " << errno << std::endl;
                throw std::runtime_error("Failed to set file descriptor flags");
            }
        }
    
        ~FileDescriptorFlagGuard() {
            // Restore original flags
            if (fcntl(fd_, F_SETFL, original_flags_) == -1) {
                std::cerr << "fcntl F_SETFL restore error: " << errno << std::endl;
                // Log error but don’t throw; destructor shouldn’t propagate exceptions
            }
        }
    
        // Prevent copying to avoid double flag management
        FileDescriptorFlagGuard(const FileDescriptorFlagGuard&) = delete;
        FileDescriptorFlagGuard& operator=(const FileDescriptorFlagGuard&) = delete;
    
    private:
        int fd_;
        int original_flags_;
    };

class SerialPort {
public:
    SerialPort(const std::string& port, int baudRate)
        : port(port), baudRate(baudRate), fd(-1) {}

    ~SerialPort() {
        if (fd != -1) {
            close(fd);
        }
    }

    bool openPort() {
        fd = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cerr << "Failed to open serial port" << std::endl;
            return false;
        }
        configure();
        return true;
    }

    void configure() {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            std::cerr << "Error getting terminal attributes" << std::endl;
            exit(1);
        }

        cfsetospeed(&tty, baudRate);
        cfsetispeed(&tty, baudRate);
        // CLOCAL: Ignore modem control lines; CREAD: Enable receiver
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;           // No parity
        tty.c_cflag &= ~CSTOPB;           // One stop bit
        tty.c_cflag &= ~CSIZE;            // Clear character size
        tty.c_cflag |= CS8;               // 8-bit characters
        // ICANON: Enable canonical mode for line-based input; reads return full lines or nothing
        tty.c_lflag |= ICANON;
        tty.c_iflag &= ~(IXANY);          // Only XON/XOFF restarts output
        // IXON: Enable XON/XOFF output control; IXOFF: Enable XON/XOFF input control
        // Handled transparently by the OS to pause/resume I/O without program intervention
        tty.c_iflag |= (IXON | IXOFF);
        tty.c_oflag &= ~OPOST;            // Disable output processing (raw mode)
        tty.c_cc[VMIN] = 1;               // Wait for at least 1 byte line
        tty.c_cc[VTIME] = 0;              // No inter-byte timeout

        tcflush(fd, TCIFLUSH);

        // TCSANOW: Apply changes immediately
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting terminal attributes" << std::endl;
            exit(1);
        }
    }

    ssize_t writeData(const std::string& data) {
        return write(fd, data.c_str(), data.size());
    }

    ssize_t readData(char* buffer, size_t size, bool blocking = false) {
        if (blocking) {
            return read(fd, buffer, size);  // Blocks until full line since ICANON is set
        } else {
            // Temporarily set O_NONBLOCK using RAII guard
            FileDescriptorFlagGuard guard(fd, O_NONBLOCK);

            ssize_t n = read(fd, buffer, size);  // Doesn't Block

            if (n == -1 && errno == EAGAIN) {
                return 0;  // No full line ready
            }
            if (n <= 0) {  // Other errors or unexpected zero
                std::cerr << "Read error after FIONREAD: " << errno << std::endl;
                return -1;
            }
            return n;  // Full line received
        }
    
        return 0;
    }

private:
    std::string port;
    int baudRate;
    int fd;
};

class InputSourceManager {
    private:
        std::vector<std::unique_ptr<std::istream>> streams;
        std::vector<std::string> names;
        size_t currentIndex = 0;
    
    public:
        // Modified constructor to handle both interactive and file-based input
    InputSourceManager(const std::string& source, bool interactive) {
        if (interactive) {
            std::cerr << "Starting in interactive mode" << std::endl;
            streams.emplace_back(&std::cin);
            names.push_back("stdin");
        } else {
            std::cerr << "Starting in File-mode " << source << std::endl;

            auto file = std::make_unique<std::ifstream>(source);

            if (!(*file)) {
                std::cerr << "Failed to open G-code file: " << source << std::endl;
                throw std::runtime_error("File opening failed");
            }
            streams.emplace_back(std::move(file));
            names.push_back(source);
        }
    }

    explicit operator bool() const {
        // Return true if we have at least one stream and the primary stream is good
        return !streams.empty() && streams[0]->good();
    }
    
    bool getNextLine(std::string& line) {
        while (!streams.empty()) {
            rotate();  // Rotate at the beginning of each iteration
            auto& stream = *streams[currentIndex];
            bool isStdin = &stream == &std::cin;
            
            if (isStdin && std::cin.rdbuf()->in_avail() == 0) {
                if (streams.size() > 1)
                    continue;  // Always skip stdin
                else
                    return false; // If it's the only stream, return false;
            }

            if (std::getline(stream, line)) {
                if (line.empty()) {
                    continue;  // Skip empty lines
                }
                if (line.find("OpenFile ") == 0) {
                    std::string filename = line.substr(9);
                    auto newFile = std::make_unique<std::ifstream>(filename);
                    if (newFile->good()) {
                        streams.push_back(std::move(newFile));
                        names.push_back(filename);
                        std::cout << "Opened file: " << filename << std::endl;
                    } else {
                        std::cerr << "Failed to open file: " << filename << std::endl;
                    }
                    continue;
                }
                return true;  // Successful read, no need to rotate again
            }
            
            if (isStdin && std::cin.eof()) {
                streams.clear();
                names.clear();
                return false;
            }
            
            if (!isStdin) {
                streams.erase(streams.begin() + currentIndex);
                names.erase(names.begin() + currentIndex);
            }
        }
        return false;
    }
    
    private:
        void rotate() {
            currentIndex = (currentIndex + 1) % streams.size();
        }
    };

const size_t maxOutstandingCommands = 60;
size_t lineNumber = 0;
size_t commandsAcknowledged = 0;


class LineQueue {
private:
    std::deque<std::string> commandQueue;
    const size_t maxQueueSize = 300;
    size_t cursorLineNumber = 0;
    size_t cursor = 0;
public:
    void moveToLine(size_t requestedLine) {
        if (requestedLine < cursorLineNumber) {
            std::cerr << "Error: Resend request for line " << requestedLine << " is no longer available in queue" << std::endl;
            return;
        }
        auto requestedCursor = cursor + (requestedLine - cursorLineNumber);
        if (requestedCursor >= 0) {
            cursor = requestedCursor;
            cursorLineNumber = requestedLine;
        } else {
            std::cerr << "Error: Requested line " << requestedLine << " is out of range." << std::endl;
            throw std::runtime_error("Retry cannot be performed, all is lost.");
        }
    }

    void add(std::string command) {
        commandQueue.push_back(command);

        if (commandQueue.size() > maxQueueSize) {
            commandQueue.pop_front();
            cursor--;
            if (cursor < 0){
                std::cerr << "Command Sending is Hung!" << std::endl;
                throw std::runtime_error("Queue has overrun max size");
            }
        }
    }

    explicit operator bool() const {
        return cursor < commandQueue.size();
    }
    
    const std::string& get() {
        if (*this) {
            cursorLineNumber++;
            cursor++;

            return commandQueue[cursor-1];
        }

        throw std::runtime_error("Invalid Access to queue");
    }
} commandQueue;

std::string addChecksum(const std::string& command) {
    int checksum = 0;
    for (char c : command) {
        checksum ^= c;
    }
    std::ostringstream formattedCommand;

    if (lineNumber != SIZE_MAX) { // Skip a line number to reset
        formattedCommand << "N" << lineNumber << " ";
    }
    lineNumber++;

    formattedCommand << command << "*" << checksum << "\n";

    return formattedCommand.str();
}

void readSerialResponse(SerialPort& serialPort, bool blocking = false) {
    char buffer[256];
    int n = serialPort.readData(buffer, sizeof(buffer) - 1, blocking);
    if (n > 0) {
        buffer[n] = '\0';
        std::string response(buffer);
        if (response.find("ok") != std::string::npos) {
            commandsAcknowledged++;
        }
        std::smatch match;
        std::regex resendRegex("Resend: (\\d+)");
        if (std::regex_search(response, match, resendRegex)) {
            size_t requestedLine = std::stoi(match[1].str());
            commandQueue.moveToLine(requestedLine);
        }

        std::cout << response << std::endl;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <baud_rate> <gcode_file | --interactive>" << std::endl;
        return 1;
    }

    std::string serialPortName = argv[1];
    int baudRate = std::stoi(argv[2]);
    std::string inputSource = argv[3];
    bool interactiveMode = (std::string(argv[3]) == "--interactive");
    

    InputSourceManager sourceManager(inputSource, interactiveMode);

    if (!sourceManager) {
        return 1;
    }

    SerialPort serialPort(serialPortName, baudRate);
    if (!serialPort.openPort()) {
        return 1;
    }

    size_t commandsSentLast = 0;
    size_t commandsSent = 0;
    bool running = true;

    while (running) {
        std::string line;
        if ((commandsSent > commandsSentLast || commandsSent == 0) && sourceManager.getNextLine(line)) {          
            commandsSentLast = commandsSent;

            std::string formattedCommand = addChecksum(line);
            commandQueue.add(formattedCommand);
        } else {
            readSerialResponse(serialPort, true); // block if we are not reading new lines
        }

        while (commandQueue && (commandsSent - commandsAcknowledged < maxOutstandingCommands)) {
            const std::string& command = commandQueue.get();
            ssize_t bytes_written = serialPort.writeData(command);
            if (bytes_written > 0) {
                if (commandsSent == SIZE_MAX) {
                    // Smartly handle commands sent so that we don't overflow
                    commandsSent = commandsSent - commandsAcknowledged;
                    commandsSentLast = commandsSentLast - commandsAcknowledged;
                    commandsAcknowledged = 0;
                }
                commandsSent++;
            } else if (bytes_written == -1) {
                std::cerr << "Serial write error" << std::endl;
                throw std::runtime_error("Serial Write Did not Succeed");
            }
            readSerialResponse(serialPort);
        }
    }
    return 0;
}
