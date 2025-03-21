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
#include <cstring>

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
                std::cerr << "fcntl F_GETFL error: " << strerror(errno) << std::endl;
                throw std::runtime_error("Failed to get file descriptor flags");
            }
    
            // Set new flags with the specified flag added
            if (fcntl(fd_, F_SETFL, original_flags_ | flag_to_set) == -1) {
                std::cerr << "fcntl F_SETFL error: " << strerror(errno) << std::endl;
                throw std::runtime_error("Failed to set file descriptor flags");
            }
        }
    
        ~FileDescriptorFlagGuard() {
            // Restore original flags
            if (fcntl(fd_, F_SETFL, original_flags_) == -1) {
                std::cerr << "fcntl F_SETFL restore error: " << strerror(errno) << std::endl;
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
                std::cerr << "Read error after FIONREAD: " << strerror(errno) << std::endl;
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
        // Configuration for stdin
        static void configureStdin() {
            struct termios tty;
            if (tcgetattr(STDIN_FILENO, &tty) != 0) {
                std::cerr << "Warning: Error getting stdin attributes: " << strerror(errno) << std::endl;
                return;
            }
            tty.c_lflag |= ICANON;  // Line-based input
            tty.c_cc[VMIN] = 1;
            tty.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &tty) != 0) {
                std::cerr << "Warning: Error setting stdin attributes: " << strerror(errno) << std::endl;
            }
            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            if (flags == -1 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
                std::cerr << "Warning: Failed to set O_NONBLOCK on stdin: " << strerror(errno) << std::endl;
            }
        }
    
        // Open a file and return a stream, or nullptr on failure
        static std::unique_ptr<std::ifstream> openFile(const std::string& filename) {
            auto stream = std::make_unique<std::ifstream>(filename);
            if (!stream->is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return nullptr;
            }
            return stream;
        }
    
        // State
        bool interactive_;                          // Whether stdin is active
        std::unique_ptr<std::ifstream> fileStream_; // Optional file stream
    
        // Try reading a line from stdin (non-blocking)
        bool tryReadStdin(std::string& line) {
            char buffer[256];
            ssize_t n = read(STDIN_FILENO, buffer, sizeof(buffer) - 1);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return false;  // No full line available
                }
                std::cerr << "Read error from stdin: " << strerror(errno) << std::endl;
                return false;
            }
            if (n == 0) return false;  // EOF (Ctrl+D)
    
            buffer[n] = '\0';
            line = std::string(buffer);
            if (!line.empty() && line.back() == '\n') line.pop_back();
            return !line.empty();
        }
    
        // Try reading a line from the file (blocking)
        bool readFile(std::string& line) {
            if (!fileStream_ || !fileStream_->good()) {
                fileStream_.reset();  // Clear if EOF or error
                return false;
            }
            if (std::getline(*fileStream_, line) && !line.empty()) {
                return true;
            }
            fileStream_.reset();  // EOF or empty line, clear the stream
            return false;
        }
    
        // Handle "OpenFile" command
        void handleOpenFile(const std::string& line) {
            std::string filename = line.substr(9);
            auto newStream = openFile(filename);
            if (newStream) {
                fileStream_ = std::move(newStream);
                std::cout << "Opened file: " << filename << std::endl;
            }
        }
    
    public:
        InputSourceManager(const std::string& source, bool interactive) 
            : interactive_(interactive) {
            if (interactive_) {
                std::cerr << "Starting in interactive mode" << std::endl;
                configureStdin();
            }
            if (!interactive && !source.empty()) {
                std::cerr << "Starting with file: " << source << std::endl;
                fileStream_ = openFile(source);
                if (!fileStream_) {
                    throw std::runtime_error("File opening failed");
                }
            }
        }
    
        explicit operator bool() const {
            return interactive_ || fileStream_;
        }
    
        bool getNextLine(std::string& line) {
                bool gotLine = false;
    
                // Try stdin if it's our turn and we're interactive
                if (interactive_) {
                    gotLine = tryReadStdin(line);
                }
                // Try file if it's our turn or stdin failed
                
                if (fileStream_ && !gotLine) {
                    gotLine = readFile(line);
                }
    
                if (!gotLine) {
                    return false;  // No line available
                }
    
                // Check for "OpenFile" command
                if (line.find("OpenFile ") == 0) {
                    handleOpenFile(line);
                }
    
                return true;  // Got a valid line
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

    std::string addChecksum(const std::string& command, size_t lineNumber) {
        std::ostringstream formattedCommand;
    
        if (lineNumber == 0) {
            std::cerr << "Warning: Line number is 0, resetting to 1" << std::endl;
            commandQueue.push_back("M110 N1"); // Reset line number
            lineNumber++; // Next line number would be N1 
        }
    
        formattedCommand << "N" << lineNumber << " "<< command;
        
        char checksum = 0;
        for (char c : command) {
            checksum ^= c;
        }
    
        if (lineNumber == 0) {
            std::cerr << "Warning: Line number is 0, resetting to 1" << std::endl;
            commandQueue.push_back("M110 N1"); // Reset line number
            lineNumber++; // Next line number would be N1 
        }
    
        formattedCommand << "*" << checksum << "\n";
    
        return formattedCommand.str();
    }
public:
    void moveToLine(size_t requestedLine) {
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
        commandQueue.push_back(addChecksum(
            command,
            commandQueue.size() - cursor + cursorLineNumber));

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
        bool canSendCommands = (commandsSent - commandsAcknowledged < maxOutstandingCommands);

        std::string line;
        if (canSendCommands && sourceManager.getNextLine(line)) {          
            commandsSentLast = commandsSent;
            commandQueue.add(line);
        } else {
            readSerialResponse(serialPort, true); // block if we are not reading new lines
        }

        while (commandQueue && canSendCommands) {
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
