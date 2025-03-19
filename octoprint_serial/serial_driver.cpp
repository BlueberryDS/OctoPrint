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
        configure(false);
        serialBlocks = false;
        return true;
    }

    void configure(bool blocking) {
        if (serialBlocks == blocking){
          return;
        }
      
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            std::cerr << "Error getting terminal attributes" << std::endl;
            exit(1);
        }

        cfsetospeed(&tty, baudRate);
        cfsetispeed(&tty, baudRate);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_lflag = 0;
        tty.c_iflag &= ~(IXANY);
        tty.c_iflag |= (IXON | IXOFF);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = blocking ? 1 : 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting terminal attributes" << std::endl;
            exit(1);
        }
        fcntl(fd, F_SETFL, blocking ? 0 : O_NONBLOCK);
    }

    ssize_t writeData(const std::string& data) {
        return write(fd, data.c_str(), data.size());
    }

    ssize_t readData(char* buffer, size_t size) {
        return read(fd, buffer, size);
    }

private:
    std::string port;
    int baudRate;
    int fd;
    bool serialBlocks;
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
            streams.emplace_back(&std::cin);
            names.push_back("stdin");
        } else {
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
                if (std::getline(*streams[currentIndex], line)) {
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
                        return getNextLine(line); // Skip OpenFile command
                    }
                    rotate();
                    return true;
                }
                // Remove exhausted stream if it's not the primary (index 0)
                if (currentIndex > 0) {
                    streams.erase(streams.begin() + currentIndex);
                    names.erase(names.begin() + currentIndex);
                } else {
                    rotate();
                }
            }
            return false;
        }
    
    private:
        void rotate() {
            currentIndex = (currentIndex + 1) % streams.size();
        }
    };

std::deque<std::string> commandQueue;
bool running = true;
const size_t maxQueueSize = 300;
const size_t maxOutstandingCommands = 60;
size_t lineNumber = 0;
size_t commandsSent = 0;
size_t commandsAcknowledged = 0;
size_t cursorLineNumber = 0;
std::deque<std::string>::iterator cursor;

std::string addChecksum(const std::string& command) {
    int checksum = 0;
    for (char c : command) {
        checksum ^= c;
    }
    std::ostringstream formattedCommand;
    formattedCommand << "N" << lineNumber << " " << command << "*" << checksum << "\n";
    lineNumber++;
    return formattedCommand.str();
}

void handleResendRequest(int requestedLine) {
    if (requestedLine < cursorLineNumber) {
        std::cerr << "Error: Resend request for line " << requestedLine << " is no longer available in queue" << std::endl;
        return;
    }
    auto requestedCursor = cursor + (requestedLine - cursorLineNumber);
    if (requestedCursor >= commandQueue.begin()) {
        cursor = requestedCursor;
        cursorLineNumber = requestedLine;
    } else {
        std::cerr << "Error: Requested line " << requestedLine << " is out of range." << std::endl;
    }
}

void readSerialResponse(SerialPort& serialPort) {
    char buffer[256];
    int n = serialPort.readData(buffer, sizeof(buffer) - 1);
    if (n > 0) {
        buffer[n] = '\0';
        std::string response(buffer);
        if (response.find("ok") != std::string::npos) {
            commandsAcknowledged++;
        }
        std::smatch match;
        std::regex resendRegex("Resend (\\d+)");
        if (std::regex_search(response, match, resendRegex)) {
            int requestedLine = std::stoi(match[1].str());
            handleResendRequest(requestedLine);
        }
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

    cursor = commandQueue.begin();
    cursorLineNumber = 0;
    int commandsSentLast = -1;
    while (running) {
        if (commandsSent > commandsSentLast) {          
            commandsSentLast = commandsSent;
            std::string line;
            if (!sourceManager.getNextLine(line)) {
                running = false;
                break;
            }
            std::string formattedCommand = addChecksum(line);
            commandQueue.push_back(formattedCommand);

            if (commandQueue.size() > maxQueueSize) {
                commandQueue.pop_front();
            }
        } else {
            serialPort.configure(true);
        }

        while (cursor != commandQueue.end() && (commandsSent - commandsAcknowledged < maxOutstandingCommands)) {
            serialPort.configure(false);
            std::string& command = *cursor;
            ssize_t bytes_written = serialPort.writeData(command);
            if (bytes_written > 0) {
                commandsSent++;
                cursorLineNumber++;
                ++cursor;
            } else if (bytes_written == -1 && errno != EAGAIN) {
                std::cerr << "Serial write error" << std::endl;
                running = false;
                break;
            }
            readSerialResponse(serialPort);
        }
        readSerialResponse(serialPort);
    }
    return 0;
}
