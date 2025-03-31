#include <iostream>
#include <stdio.h>
#include <string>
#include <deque>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <errno.h>
#include <regex>
#include <cstring>
#include <thread>
#include <chrono>
#include <array>
#include <algorithm>
#include <charconv>

// Structure to hold parsed arguments
struct Args {
    std::string serialPortName;
    int baudRate = 0;
    std::string inputSource;
    bool interactiveMode = false;
    bool verbose = false;
    bool sendBusy = false;
    int serialBufferSize = 128;
    long closePosition = 0;
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
    // Define a type alias for the FILE* smart pointer with fclose as deleter
    using FilePtr = std::unique_ptr<FILE, int(*)(FILE*)>;
    FilePtr fileStream_{nullptr, std::fclose}; // Initialize with nullptr and fclose
    const Args& args_;
    std::chrono::time_point<std::chrono::steady_clock> lastBusyTime_;
    std::streampos totalFileSize_ = 0;

    // Remove comments from a line (unchanged)
    void stripComments(std::string& line) {
        bool inParentheses = false;
        size_t writePos = 0;
    
        // Filter the string in place
        for (size_t readPos = 0; readPos < line.length(); ++readPos) {
            if (line[readPos] == '(') {
                inParentheses = true;
                continue;
            }
            if (line[readPos] == ')') {
                inParentheses = false;
                continue;
            }
            if (line[readPos] == ';' && !inParentheses) {
                break;  // Stop at semicolon outside parentheses
            }
            if (!inParentheses) {
                line[writePos++] = line[readPos];
            }
        }
    
        // Handle empty or fully whitespace cases
        if (writePos == 0) {
            line.clear();
            return;
        }
    
        // Trim leading whitespace
        size_t start = 0;
        while (start < writePos && (line[start] == ' ' || line[start] == '\t')) {
            ++start;
        }
    
        // Trim trailing whitespace
        size_t end = writePos - 1;
        while (end > start && (line[end] == ' ' || line[end] == '\t')) {
            --end;
        }
    
        // If all whitespace, clear the string
        if (start > end) {
            line.clear();
            return;
        }

        // Remove trailing whitespace
        line.erase(end < writePos - 1 ? end + 1 : writePos);
        line.erase(0, start);
    }

    // Configure stdin for non-blocking, line-based input (unchanged)
    void configureStdin() {
        struct termios tty;
        if (tcgetattr(STDIN_FILENO, &tty) != 0) {
            std::cerr << "Warning: Error getting stdin attributes: " << strerror(errno) << std::endl;
            return;
        }
        tty.c_lflag |= ICANON;
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

    // Open a file using C-style functions and return a FilePtr
    FilePtr openFile(const std::string& filename) {
        FILE* fp = fopen(filename.c_str(), "r");

        if (!fp) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return FilePtr(nullptr, std::fclose);
        }

        int fd = fileno(fp);
        if (flock(fd, LOCK_SH) != 0) {  // Shared lock for reading
            std::cerr << "Failed to lock file: " << filename << std::endl;
            fclose(fp);
            return FilePtr(nullptr, std::fclose);;
        }

        fseek(fp, 0, SEEK_END); // Move to the end of the file
        totalFileSize_ = ftell(fp); // Get the file size
        fseek(fp, 0, SEEK_SET); // Reset to the beginning of the file

        if (args_.closePosition > 0) {
            fseek(fp, args_.closePosition, SEEK_SET); // Move to the specified position
            if (ftell(fp) != args_.closePosition) {
                std::cerr << "Failed to seek to close position: " << args_.closePosition << std::endl;
                fclose(fp);
                return FilePtr(nullptr, std::fclose);
            }
        }

        return FilePtr(fp, std::fclose);
    }

    bool tryReadStdin(std::string& line) {
        line.resize(line.capacity());
        ssize_t n = read(STDIN_FILENO, &line[0], line.size() - 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return false;
            }
            std::cerr << "Read error from stdin: " << strerror(errno) << std::endl;
            return false;
        }
        if (n == 0) return false;
        
        if (line[n - 1] == '\n') --n; // Remove newline if present
        line.resize(n);
        return !line.empty();
    }
    
    bool readFile(std::string& line) {
        sendBusyMessageIfNeeded();
        if (!fileStream_) {
            return false;
        }

        bool readFullLine = true;

        line.resize(line.capacity());
        while (auto ptr = fgets(&line[0], line.size(), fileStream_.get())) {
            size_t len = strlen(ptr);
            if (len > 0 && line[len - 1] != '\n') {
                readFullLine = false;
                continue; // Not a full line yet
            }
            
            line.resize(len-1); // Remove newline
            return readFullLine;
        }

        if (feof(fileStream_.get())) {
            std::cerr << "End of file reached" << std::endl;
        } else {
            std::cerr << "File read error: " << strerror(errno) << std::endl;
        }
        
        fileStream_.reset(); // Clear stream on EOF or error
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

    // Get file progress using ftell
    std::string getFileProgress() {
        if (fileStream_) {
            auto currentPos = ftell(fileStream_.get());
            return std::to_string(currentPos) + "/" + std::to_string(totalFileSize_);
        }
        return "0/0";
    }

    // Send busy message if needed (unchanged)
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
    InputSourceManager(const Args& args) : args_(args) {
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

    ~InputSourceManager() {
        if (fileStream_) {
            auto currentPos = ftell(fileStream_.get());
            std::cerr << "Closing file stream at position: " << currentPos << std::endl;
            fileStream_.reset(); // Close file stream
        }
    }

    explicit operator bool() const {
        return args_.interactiveMode || fileStream_;
    }

    bool getNextLine(std::string& line) {
        bool gotLine = false;
        while (!gotLine) {
            if (args_.interactiveMode) {
                gotLine = tryReadStdin(line);
                if (gotLine && !args_.verbose) {
                    std::cout << "ok" << std::endl;
                }
            }
            if (fileStream_ && !gotLine) {
                gotLine = readFile(line);
            }

            if (!gotLine) {
                return false;
            }

            stripComments(line);
            if (line.empty()) {
                gotLine = false;
                continue;
            }

            if (line.find("OpenFile ") == 0) {
                handleOpenFile(line);
                return false;
            }
        }
        return true;
    }

    void closeFile() {
        if (fileStream_) {
            auto currentPos = ftell(fileStream_.get());
            std::cerr << "Closing file stream at position: " << currentPos << std::endl;
            fileStream_.reset(); // Close file stream
        }
    }
};
    
class CommandQueue {
    std::array<std::string, 300> buffer;  // Fixed-size ring buffer with 100 slots
    size_t start = 0;                     // Index of the oldest command
    size_t size = 0;                      // Number of commands currently in the queue
    size_t cursor = 0;                    // Logical index of the next command to process
    size_t cursorLineNumber = 0;          // Line number of the last processed command

public:
    // **Constructor**: Initializes the buffer and preallocates string capacities
    CommandQueue() {
        for (auto& str : buffer) {
            str.reserve(128);  // Reserve 128 characters per string to avoid reallocations
        }
    }

    // **moveToLine**: Moves the cursor to a specific line number
    size_t moveToLine(size_t requestedLine) {
        if (requestedLine > cursorLineNumber) {
            std::cerr << "Error: Requested line " << requestedLine 
                        << " is ahead of current line " << cursorLineNumber << std::endl;
            throw std::runtime_error("Retry cannot be performed, all is lost.");
        }

        size_t rewindRequested = cursorLineNumber - requestedLine;
        if (rewindRequested > cursor) {
            std::cerr << "Error: Requested line " << requestedLine << " is out of range." << std::endl;
            throw std::runtime_error("Retry cannot be performed, all is lost.");
        }

        cursor -= rewindRequested;
        cursorLineNumber = requestedLine;
        return rewindRequested;
    }

    // **nextLineNumber**: Returns the line number for the next command to be added
    size_t nextLineNumber() {
        return size - cursor + cursorLineNumber;
    }

    // **next**: Returns a reference to the next available string for writing a command
    std::string& next() {
        size_t writeIndex = (start + size) % 100;
        return buffer[writeIndex];
    }

    // **commit**: Commits the new command to the queue
    void commit() {
        if (size < 100) {
            size++;  // Add new command if buffer is not full
        } else {
            // Buffer is full: overwrite the oldest command
            start = (start + 1) % 100;  // Move start to the next position
            if (cursor > 0) {
                cursor--;  // Adjust cursor to maintain logical position
            } else {
                std::cerr << "Command Sending is Hung!" << std::endl;
                throw std::runtime_error("Queue has overrun max size");
            }
        }
    }

    // **operator bool**: Checks if there are more commands to process
    operator bool() const {
        return cursor < size;
    }

    // **currentLineNumber**: Returns the current line number
    size_t currentLineNumber() const {
        return cursorLineNumber;
    }

    // **peekCursor**: Returns a const reference to the next command without advancing
    const std::string& peekCursor() const {
        if (cursor < size) {
            size_t physicalIndex = (start + cursor) % 100;
            return buffer[physicalIndex];
        } else {
            throw std::out_of_range("Cursor out of range in peekCursor");
        }
    }

    // **getCursor**: Returns the next command and advances the cursor
    std::string& getCursor() {
        if (cursor < size) {
            size_t physicalIndex = (start + cursor) % 100;
            cursor++;           // Advance cursor to the next command
            cursorLineNumber++; // Increment line number after processing
            return buffer[physicalIndex];
        } else {
            throw std::out_of_range("Cursor out of range in getCursor");
        }
    }
};

class LineQueue {
private:
    const size_t maxLineLength = 96;
    size_t serialBufferSize = 128;
    size_t bytesSentSinceLastOK = 0;
    int asciiBufferSize = 4;
    CommandQueue commandQueue;
    size_t lineAcknowledged = 0;
    int maxStepper = 0;
    const char* M110 = "M110"; // with checksum
    size_t resendsToIgnore = 0;
    size_t lastRequestedResendLine = 0;
    const size_t acknowledgeMissedAllowance = 10;

    void addChecksum(std::string& command, size_t lineNumber) {
        std::string lineNumberStr = "N" + std::to_string(lineNumber) + " ";
        command.insert(0, lineNumberStr);

        int checksum = 0;
        for (char c : command) {
            checksum ^= c;
        }

        command.push_back('*');
        command.append(std::to_string(checksum));
        command.push_back('\n');

        if (command.size() > maxLineLength) {
            std::cerr << "Error: " << command << " is too long" << std::endl;
            command = "M118 Too Long"; // kill any lines longer than max length
            addChecksum(command, lineNumber);
        }
    }
public:
    LineQueue (const Args& args) : serialBufferSize(args.serialBufferSize) {}

    void moveToLine(size_t requestedLine) {
        std::cerr << "Retry requested for line " << requestedLine 
              << ". Currently on line " << commandQueue.currentLineNumber() << std::endl;

        if (resendsToIgnore && requestedLine == lastRequestedResendLine) {
            resendsToIgnore--; // Since we pack the buffer, the firmware will send multiple resends
            return;
        } else if (resendsToIgnore && requestedLine < lastRequestedResendLine) { // This misses an error condition when the lines overflow, but assume unlikely
            std::cerr << "Error: Resend " << requestedLine << " requested which less than previous resend" << std::endl;
            throw std::runtime_error("Retry cannot be performed, all is lost.");
        } // If a resend is issued for a further line, assume we've passed the previous retry
        
        resendsToIgnore = commandQueue.moveToLine(requestedLine);
        lastRequestedResendLine = requestedLine;
    }

    void commit() {
        addChecksum(commandQueue.next(), commandQueue.nextLineNumber());
        commandQueue.commit();
    }

    std::string& next() {
        auto& line = commandQueue.next();
        auto lineNumber = commandQueue.nextLineNumber();
        if (lineNumber == 0) {
            std::cerr << "Warning: Line number is 0, resetting to 0" << std::endl;
            line = M110;
            addChecksum(line, lineNumber); // Reset line number
            commandQueue.commit();
        }

        return commandQueue.next();
    }

    explicit operator bool() const {
        return commandQueue;
    }

    bool canSendCommands() const {
        // Both the queue and the serial buffer need to have space
        return *this
                // reserve a portion of the ascii buffer for injected commands
                && (commandQueue.currentLineNumber() - lineAcknowledged < asciiBufferSize * 0.75) 
                 // make sure we aren't overrunning the buffer
                && (serialBufferSize * 0.75 - bytesSentSinceLastOK > commandQueue.peekCursor().size())
                // If we are currently in resend status, give it a chance to catch up.
                // We do this because buffer size is unknowable while in resend status.
                && (!lastRequestedResendLine || commandQueue.currentLineNumber() - lineAcknowledged <= 1); 
    }

    void acknowledge(size_t lineNo, int stepperBuffer, int asciiBuffer) {
        // Ignore out-of-order acknowledgments in case we have some sort of bug.
        if (lineNo < lineAcknowledged || lineNo > lineAcknowledged + acknowledgeMissedAllowance) {
            std::cerr << "Error: Acknowledged line " << lineNo 
                      << " does not follow the current previous ack " << lineAcknowledged << std::endl;
            return;
        }

        lineAcknowledged=lineNo;

        if (lineNo >= lastRequestedResendLine) { // If we've successfully processed the resend, then we can reset the resend counter
            lastRequestedResendLine = 0;
            resendsToIgnore = 0;
            bytesSentSinceLastOK = 0;
        }

        // make sure we don't screw up our buffer settings due to some late-stage
        // serial error
        if (commandQueue.currentLineNumber() < 1000) {
            if (stepperBuffer > maxStepper) {
            std::cerr << "Setting stepper buffer size to " << stepperBuffer << std::endl;
            maxStepper = stepperBuffer;
            }

            if (asciiBufferSize < asciiBuffer) {
            std::cerr << "Setting ascii buffer size to " << asciiBuffer << std::endl;
            asciiBufferSize = asciiBuffer;
            }
        }

        if (asciiBuffer > asciiBufferSize * 0.9) {
            static char bufferCounter = 0;
            if (bufferCounter++ % 10 == 0) {
                std::cerr << "Warning: Ascii buffer is almost empty" << std::endl;
            }
        }
        if (stepperBuffer > maxStepper * 0.9) {
            static char bufferCounter = 0;
            if (bufferCounter++ % 10 == 0) {
                std::cerr << "Warning: Stepper buffer is almost empty" << std::endl;
            }
        }
        
       
    }
    
    const std::string& get() {
        auto & ret = commandQueue.getCursor();

        bytesSentSinceLastOK += ret.size();

        return ret;
    }
};


template <typename T = int>
T extractNumberAfterPrefix(const std::string& str, const std::string& prefix) {
    size_t pos = str.find(prefix);
    if (pos != std::string::npos) {
        pos += prefix.length();
        while (pos < str.size() && std::isspace(str[pos])) ++pos;
        size_t end = pos;
        while (end < str.size() && std::isdigit(str[end])) ++end;
        if (end - pos > 0 && end - pos <= 10) { // Check if the number is within a reasonable range
            T value = 0;
            auto [ptr, ec] = std::from_chars(str.data() + pos, str.data() + end, value);
            if (ec == std::errc::invalid_argument) {
                std::cerr << "Error: Invalid argument while converting string to number" << std::endl;
                return static_cast<T>(-1);
            } else if (ec == std::errc::result_out_of_range) {
                std::cerr << "Error: Number out of range while converting string to number" << std::endl;
                return static_cast<T>(-1);
            }
            return value;
        }
    }
    return static_cast<T>(-1);
}

void readSerialResponse(SerialPort& serialPort, LineQueue & queue, InputSourceManager & inputManager, bool blocking, const Args& args) {
    static char buffer[256];
    static std::string response;
    response.assign(buffer);

    int n = serialPort.readData(buffer, sizeof(buffer) - 1, blocking);
    if (n > 0) {
        buffer[n] = '\0';

        if (response.find("ok", 0) == 0) { // Check if "ok" is at the start of the line
            int lineNo = extractNumberAfterPrefix(response, "ok N");
            int stepperBuffer = extractNumberAfterPrefix(response, "P");
            int asciiBuffer = extractNumberAfterPrefix(response, "B");

            if (lineNo != -1) {
            queue.acknowledge(lineNo, stepperBuffer, asciiBuffer);
            }

            if (args.verbose) {
            std::cout << response;
            }
        }
        else if (response.find("Resend: ", 0) == 0) { // Check if "Resend: " is at the start of the line
            int requestedLine = extractNumberAfterPrefix(response, "Resend: ");
            if (requestedLine != -1) {
            queue.moveToLine(requestedLine);
            }
            if (args.verbose) {
            std::cout << response;
            }
        }
        else if (response.find("//action:cancel", 0) == 0) { // Check if "//action:cancel" is at the start of the line
            std::cerr << "Cancel action received. Terminating process." << std::endl;
            inputManager.closeFile();
            std::cout << response;
        }
        else {
            std::cout << response;
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
              << "  --sendbusy           Enable Fake Busy signals when file processing\n"
              << "  --serialbuffersize   Set the serial buffer size (default: 128)\n"
              << "  --closeposition      Set the close position (default: 0)\n";
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
        args.baudRate = std::stol(argv[2]);
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
        } else if (arg == "--serialbuffersize") {
            if (i + 1 < argc) {
                try {
                    args.serialBufferSize = std::stol(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid serial buffer size '" << argv[i] << "': " << e.what() << std::endl;
                    printUsage(argv[0]);
                    throw;
                }
            } else {
                std::cerr << "Error: --serialbuffersize requires a value\n";
                printUsage(argv[0]);
                throw std::runtime_error("Missing value for --serialbuffersize");
            }
        } else if (arg == "--closeposition") {
            if (i + 1 < argc) {
                try {
                    args.closePosition = std::stoul(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid close position '" << argv[i] << "': " << e.what() << std::endl;
                    printUsage(argv[0]);
                    throw;
                }
            } else {
                std::cerr << "Error: --closeposition requires a value\n";
                printUsage(argv[0]);
                throw std::runtime_error("Missing value for --closeposition");
            }
        } else {
            std::cerr << "Error: Unknown option '" << arg << "'" << std::endl;
            printUsage(argv[0]);
            throw std::runtime_error("Unknown option");
        }
    }

    return args;
}

void processCommandQueue(SerialPort& serialPort, LineQueue& queue, InputSourceManager& inputSourceManager ,const Args& args) {
    while (queue.canSendCommands()) {
        const std::string& command = queue.get();
        ssize_t bytes_written = serialPort.writeData(command);
        if (bytes_written == -1) {
            std::cerr << "Serial write error" << std::endl;
            throw std::runtime_error("Serial Write Did not Succeed");
        }

        if (args.verbose) {
            std::cout << command << '\n';
        }
            
        readSerialResponse(serialPort, queue, inputSourceManager, false, args);
    }
}

int main(int argc, char* argv[]) {
    try {
        Args args;

        args = parseArguments(argc, argv);

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

        LineQueue commandQueue(args);

        commandQueue.next() = "M115"; // Start with an initial GCode command to coordinate the line numbers
        commandQueue.commit();
        processCommandQueue(serialPort, commandQueue, sourceManager, args);


        bool running = true;

        while (running) {
            if (!commandQueue && sourceManager.getNextLine(commandQueue.next())) {          
                commandQueue.commit();
            } else {
                readSerialResponse(serialPort, commandQueue, sourceManager, true, args); // block if we are not reading new lines
            }
            
            processCommandQueue(serialPort, commandQueue, sourceManager, args);
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << " (Exception type: " << typeid(e).name() << ")" << std::endl;
        return 1;
    }
}
