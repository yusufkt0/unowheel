#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <hidapi/hidapi.h>
#include <string>
#include <sstream>
#include <cstring> // For memset
#include <cstdint> // For uint8_t, uint32_t

// Serial Port Settings
const char* portName = "/dev/ttyUSB0"; // Change as necessary
const int baudRate = B9600;

// HID Report Descriptor (example for a simple joystick)
const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,     // Usage Page (Generic Desktop)
    0x09, 0x04,     // Usage (Joystick)
    0xa1, 0x01,     // Collection (Application)
    0x05, 0x01,     // Usage Page (Generic Desktop)
    0x09, 0x01,     // Usage (Pointer)
    0xa1, 0x00,     // Collection (Physical)
    0x09, 0x30,     // Usage (X)
    0x09, 0x31,     // Usage (Y)
    0x15, 0x00,     // Logical Minimum (0)
    0x26, 0xff, 0x00, // Logical Maximum (255)
    0x75, 0x08,     // Report Size (8)
    0x95, 0x02,     // Report Count (2)
    0x81, 0x02,     // Input (Data, Var, Abs)
    0xc0,           // End Collection
    0x05, 0x09,     // Usage Page (Button)
    0x19, 0x01,     // Usage Minimum (Button 1)
    0x29, 0x10,     // Usage Maximum (Button 16)
    0x15, 0x00,     // Logical Minimum (0)
    0x25, 0x01,     // Logical Maximum (1)
    0x75, 0x01,     // Report Size (1)
    0x95, 0x10,     // Report Count (16)
    0x81, 0x02,     // Input (Data, Var, Abs)
    0xc0            // End Collection
};

// Function prototypes
bool SetupSerialPort(int &fd);
bool ReadSerialData(int fd, std::string &data);
void SendHIDReport(hid_device *handle, uint8_t *report, size_t report_size);

int main(int argc, char* argv[]) {
    // Open serial port
    int fd;
    if (!SetupSerialPort(fd)) {
        std::cerr << "Error: Could not open serial port." << std::endl;
        return 1;
    }

    // Initialize HIDAPI
    if (hid_init()) {
        std::cerr << "Error: HIDAPI initialization failed." << std::endl;
        close(fd);
        return 1;
    }

    // Open HID device
    hid_device *handle = hid_open(0x258a,0x002a, nullptr); // Replace with your device's Vendor ID and Product ID
    if (!handle) {
        std::cerr << "Error: Could not open HID device." << std::endl;
        close(fd);
        hid_exit();
        return 1;
    }

    // Main loop
    std::string serialData;
    uint8_t report[4] = {0}; // Example report size, adjust based on your descriptor
    while (true) {
        if (ReadSerialData(fd, serialData)) {
            // Process the serial data
            std::istringstream ss(serialData);
            std::string token;
            while (std::getline(ss, token, ',')) {
                if (token[0] == 'B') {
                    int index = std::stoi(token.substr(1, 1));
                    int state = std::stoi(token.substr(3, 1));
                    // Simulate button press/release
                    if (state == 0) {
                        report[2] &= ~(1 << index); // Clear bit
                    } else {
                        report[2] |= (1 << index);  // Set bit
                    }
                } else if (token[0] == 'A') {
                    int index = std::stoi(token.substr(1, 1));
                    int value = std::stoi(token.substr(3));
                    // Map analog value to axis (0-255 range for simplicity)
                    report[index] = static_cast<uint8_t>((value / 1023.0) * 255);
                }
            }
            SendHIDReport(handle, report, sizeof(report));
        }
        usleep(100000); // Delay to avoid excessive CPU usage
    }

    // Clean up
    hid_close(handle);
    close(fd);
    hid_exit();
    return 0;
}

bool SetupSerialPort(int &fd) {
    fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error: Could not open serial port." << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error: Could not get serial port attributes." << std::endl;
        close(fd);
        return false;
    }

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN]  = 0; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error: Could not set serial port attributes." << std::endl;
        close(fd);
        return false;
    }
    return true;
}

bool ReadSerialData(int fd, std::string &data) {
    char buffer[1024];
    int bytesRead = read(fd, buffer, sizeof(buffer) - 1);
    if (bytesRead > 0) {
        buffer[bytesRead] = '\0';
        data = std::string(buffer);
        return true;
    }
    return false;
}

void SendHIDReport(hid_device *handle, uint8_t *report, size_t report_size) {
    int res = hid_write(handle, report, report_size);
    if (res < 0) {
        std::cerr << "Error: Failed to send HID report. " << hid_error(handle) << std::endl;
    }
}

