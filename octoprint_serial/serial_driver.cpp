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
#include <thread>
#include <chrono>

// Structure to hold parsed arguments
struct Args {
    std::string serialPortName;
    int baudRate = 0;
    std::string inputSource;
    bool interactiveMode = false;
    bool verbose = false;
    bool sendBusy = false;
};

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
    SerialPort(const Args& args)
        : port(args.serialPortName), baudRate(args.baudRate), fd(-1) {}

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
        tty.c_cflag |= CSTOPB;            // Two stop bits
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
        std::string stripComments(const std::string& line) {
            std::string result;
            bool inParentheses = false;
            
            for (size_t i = 0; i < line.length(); ++i) {
                if (line[i] == '(') {
                    inParentheses = true;
                    continue;
                }
                if (line[i] == ')') {
                    inParentheses = false;
                    continue;
                }
                if (line[i] == ';' && !inParentheses) {
                    break;  // Stop at semicolon if not in parentheses
                }
                if (!inParentheses) {
                    result += line[i];
                }
            }
            
            // Remove leading and trailing whitespace
            size_t start = result.find_first_not_of(" \t");
            size_t end = result.find_last_not_of(" \t");
            
            if (start == std::string::npos) {
                return "";  // Empty line after stripping
            }
            
            return result.substr(start, end - start + 1);
        }

        // Configuration for stdin
        void configureStdin() {
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
        std::unique_ptr<std::ifstream> openFile(const std::string& filename) {
            auto stream = std::make_unique<std::ifstream>(filename);
            if (!stream->is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return nullptr;
            }
            // Save total file size
            stream->seekg(0, std::ios::end);
            totalFileSize_ = stream->tellg();
            stream->seekg(0, std::ios::beg);

            return stream;
        }
    
        // State
        std::unique_ptr<std::ifstream> fileStream_; // Optional file stream
        const Args& args_;
        std::chrono::time_point<std::chrono::steady_clock> lastBusyTime_;
        std::streampos totalFileSize_ = 0;

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
            sendBusyMessageIfNeeded();

            if (!fileStream_ || !fileStream_->good()) {
                fileStream_.reset();  // Clear if EOF or error
                return false;
            }
            if (std::getline(*fileStream_, line) && !line.empty()) {
                return true;
            }
            return false;
        }
    
        // Handle "OpenFile" command
        void handleOpenFile(const std::string& line) {
            std::string filename = line.substr(9);
            auto newStream = openFile(filename);
            if (newStream) {
                fileStream_ = std::move(newStream);
                std::cerr << "Opened file: " << filename << std::endl;
                if (args_.sendBusy) {
                    lastBusyTime_ = std::chrono::steady_clock::now();
                }
            }
        }

        // Get file progress
        std::string getFileProgress() {
            if (fileStream_) {
                auto currentPos = fileStream_->tellg();
                return std::to_string(currentPos) + "/" + std::to_string(totalFileSize_);
            }
            return "0/0";
        }

        // Send busy message if needed
        void sendBusyMessageIfNeeded() {
            if (args_.sendBusy) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - lastBusyTime_).count() >= 2) {
                    std::cout << "echo:busy: Printing from File" << std::endl;
                    std::cout << "File Progress " << getFileProgress() << std::endl;
                    lastBusyTime_ = now;
                }
            }
        }
    
    public:
        InputSourceManager(const Args& args) 
            :args_(args) {
            if (args.interactiveMode) {
                std::cerr << "Starting in interactive mode" << std::endl;
                configureStdin();
            }
            if (!args.interactiveMode && !args.inputSource.empty()) {
                std::cerr << "Starting with file: " << args.inputSource << std::endl;
                fileStream_ = openFile(args.inputSource);
                if (!fileStream_) {
                    throw std::runtime_error("File opening failed");
                }
                if (args.sendBusy) {
                    lastBusyTime_ = std::chrono::steady_clock::now();
                }
            }
        }
    
        explicit operator bool() const {
            return args_.interactiveMode || fileStream_;
        }
    
        bool getNextLine(std::string& line) {
            bool gotLine = false;

            while (!gotLine) {
                // Try stdin if it's our turn and we're interactive
                if (args_.interactiveMode) {
                    gotLine = tryReadStdin(line);
                    if (gotLine && !args_.verbose) {
                        // Fake OK for all stdinputs in non-verbose mode
                        std::cout << "ok" << std::endl;
                    }
                }
                // Try file if it's our turn or stdin failed
                
                if (fileStream_ && !gotLine) {
                    gotLine = readFile(line);
                }
    
                if (!gotLine) {
                    return false;  // No line available
                }

                line = stripComments(line);

                if(line.empty()) {
                    continue;  // Skip Empty Lines
                }
    
                // Check for "OpenFile" command
                if (line.find("OpenFile ") == 0) {
                    handleOpenFile(line);
                    return false;
                }
            }
            return true;  // Got a valid line
        }
    };

class LineQueue {
private:
    const size_t maxOutstandingCommands = 30;
    std::deque<std::string> commandQueue;
    const size_t maxQueueSize = 100;
    size_t cursorLineNumber = 0;
    size_t cursor = 0;
    size_t lineAcknowledged = 0;
    size_t interbuffer = 0; // Estimate of how many requests are in the serial buffer only
    const char* M110 = "M110"; // with checksum

    std::string addChecksum(const std::string& command, size_t lineNumber) {
        std::ostringstream formattedCommand;
    
        formattedCommand << "N" << lineNumber << " "<< command;
        
        int checksum = 0;
        for (char c : formattedCommand.str()) {
            checksum ^= c;
        }
    
        formattedCommand << "*" << checksum << "\n";
    
        return formattedCommand.str();
    }
public:
    void moveToLine(size_t requestedLine) {
        if (requestedLine > cursorLineNumber) {
            std::cerr << "Error: Requested line " << requestedLine << " is ahead of current line " << cursorLineNumber << std::endl;
            throw std::runtime_error("Retry cannot be performed, all is lost.");
        }
        
        auto rewindRequested = (cursorLineNumber - requestedLine);
        interbuffer = rewindRequested; // Rewind status means we have stuff in the interbuffer for sure
        auto requestedCursor = cursor - rewindRequested;
        if (requestedCursor >= 0) {
            cursor = requestedCursor;
            cursorLineNumber = requestedLine;
            std::cerr << "Resending from line " << requestedLine << std::endl;
        } else {
            std::cerr << "Error: Requested line " << requestedLine << " is out of range." << std::endl;
            throw std::runtime_error("Retry cannot be performed, all is lost.");
        }
    }

    void add(std::string command) {
        auto lineNumber = commandQueue.size() - cursor + cursorLineNumber;
        if (lineNumber == 0) {
            std::cerr << "Warning: Line number is 0, resetting to 0" << std::endl;
            commandQueue.push_back(addChecksum(M110, lineNumber)); // Reset line number
            lineNumber++; // Next line number would be N1 
        }

        commandQueue.push_back(addChecksum(
            command,
            lineNumber)); // Calculate line number at end of queue

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

    bool canSendCommands() const {
        // Subtract to interbuffer-amount from the maxOutstandingCommands to try to avoid
        // overrunning the serial buffer, but always allow one extra command to be
        // sent so we can actually fill the ascii buffer.
        return (cursorLineNumber - lineAcknowledged <= maxOutstandingCommands - interbuffer);
    }

    void acknowledge(size_t lineNo, int stepperBuffer, int asciiBuffer) {
        lineAcknowledged=lineNo;

        // Read the ascii buffer size to estimate how many commands are in the serial buffer
        if(asciiBuffer >= 0) {
            interbuffer = cursorLineNumber - lineAcknowledged - (maxOutstandingCommands - asciiBuffer);
        }
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

void readSerialResponse(SerialPort& serialPort, bool blocking, const Args& args) {
    char buffer[256];
    int n = serialPort.readData(buffer, sizeof(buffer) - 1, blocking);
    if (n > 0) {
        buffer[n] = '\0';
        std::smatch match;
        std::regex resendRegex("Resend: (\\d+)");

        std::string response(buffer);
        if (response.find("ok") != std::string::npos) {
            std::regex okRegex("ok N(\\d+)");
            if (std::regex_search(response, match, okRegex)) {
                int lineNo = std::stoi(match[1].str());
                std::regex pRegex("P(\\d+)");
                std::regex bRegex("B(\\d+)");
                int stepperBuffer = -1; // Signifies not sent
                int asciiBuffer = -1;

                if (std::regex_search(response, match, pRegex)) {
                    stepperBuffer = std::stoi(match[1].str());
                }
                if (std::regex_search(response, match, bRegex)) {
                    asciiBuffer = std::stoi(match[1].str());
                }

                commandQueue.acknowledge(lineNo, stepperBuffer, asciiBuffer);
            } // Ignore extraneous OKs

            if(args.verbose) {
                std::cout << response << std::flush;
            }
        }
        else if (std::regex_search(response, match, resendRegex)) {
            size_t requestedLine = std::stoi(match[1].str());
            commandQueue.moveToLine(requestedLine);
            if(args.verbose) {
                std::cout << response << std::flush;
            }
        }
        else {
            std::cout << response << std::flush;
        }
    }
}

// Function to display usage
void printUsage(const char* programName) {
    std::cerr << "Usage: " << programName << " <serial_port> <baud_rate> <gcode_file | --interactive> [options]\n"
              << "Required arguments:\n"
              << "  <serial_port>         Serial port (e.g., /dev/ttyUSB0)\n"
              << "  <baud_rate>          Baud rate (e.g., 115200)\n"
              << "  <gcode_file | --interactive>  G-code file path or --interactive for stdin\n"
              << "Options:\n"
              << "  --verbose            Enable verbose logging\n"
              << "  --sendbusy           Enable Fake Busy signals when file processing\n";
}

// Function to parse arguments with stubs
Args parseArguments(int argc, char* argv[]) {
    Args args;

    // Check for minimum arguments or help flag
    if (argc < 4 || std::string(argv[1]) == "--help") {
        printUsage(argv[0]);
        if (argc < 4) {
            throw std::runtime_error("Insufficient arguments");
        }
        // For --help, we'll return default args after printing usage
        args.serialPortName = ""; // Invalid to force exit after help
        return args;
    }

    // Required arguments
    args.serialPortName = argv[1];
    try {
        args.baudRate = std::stoi(argv[2]);
    } catch (const std::exception& e) {
        std::cerr << "Error: Invalid baud rate '" << argv[2] << "': " << e.what() << std::endl;
        printUsage(argv[0]);
        throw;
    }
    args.inputSource = argv[3];
    args.interactiveMode = (args.inputSource == "--interactive");

    // Process optional arguments
    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--verbose") {
            args.verbose = true;
        } else if (arg == "--sendbusy") {
            args.sendBusy = true;
        } else {
            std::cerr << "Error: Unknown option '" << arg << "'\n";
            printUsage(argv[0]);
            throw std::runtime_error("Unknown option");
        }
    }

    return args;
}

int main(int argc, char* argv[]) {
    Args args;
    try {
        args = parseArguments(argc, argv);
    } catch (const std::exception& e) {
        return 1;
    }

    // Exit cleanly after --help
    if (args.serialPortName.empty()) {
        return 0;
    }

    InputSourceManager sourceManager(args);

    if (!sourceManager) {
        return 1;
    }

    SerialPort serialPort(args);
    if (!serialPort.openPort()) {
        return 1;
    }

    commandQueue.add("M115"); // Start with an initial GCode command to coordinate the line numbers

    bool running = true;

    while (running) {
        while (commandQueue && commandQueue.canSendCommands()) {
            const std::string& command = commandQueue.get();
            ssize_t bytes_written = serialPort.writeData(command);
            if (bytes_written == -1) {
                std::cerr << "Serial write error" << std::endl;
                throw std::runtime_error("Serial Write Did not Succeed");
            }
                
            readSerialResponse(serialPort, false, args);
        }

        std::string line;
        if (commandQueue.canSendCommands() && sourceManager.getNextLine(line)) {          
            commandQueue.add(line);
        } else {
            readSerialResponse(serialPort, true, args); // block if we are not reading new lines
        }        
    }
    return 0;
}
