///////////////////////////////////////////////////////////////////////////////
// main.cpp
//
// Contains implementation of the entry point for shm_server application
///////////////////////////////////////////////////////////////////////////////

#include <cassert>
#include <cstring>
#include <cstdio>

#include <stdexcept>
#include <string>

#include <ros/ros.h>

#include "shm_comm/Channel.hpp"

void print_usage()
{
    printf("Usage: ./shm_server <name> <size> <readers>\n");
    printf(" where\n");
    printf("  - <name> - unique name for created channel\n");
    printf("  - <size> - size in bytes of shared memory buffer for that channel\n");
    printf("  - <readers> - number of readers allowed to read from channel\n");
}

const char* parse_name(const char* name_str)
{
    const auto name_len = strlen(name_str);
    if(name_len == 0)
    {
        throw std::runtime_error("Name of the channel cannot be empty");
    }
    else if(const auto max_name_len = 128; name_len >= max_name_len)
    {
        throw std::runtime_error("Name of the channel must have max "
            + std::to_string(max_name_len) + " characters, got "
            + std::to_string(name_len));
    }

    return name_str;
}

int parse_size(const char* size_str)
{
    int size;
    if(const auto result = sscanf(size_str, "%d", &size); result != 1)
    {
        throw std::runtime_error("Invalid channel size: '"
            + std::string(size_str) + "'");
    }

    if(size <= 0)
    {
        throw std::runtime_error("Size of the channel must be grater than zero, got '"
            + std::string(size_str) + "'");
    }

    return size;
}

int parse_readers(const char* readers_str)
{
    int readers;
    if(const auto result = sscanf(readers_str, "%d", &readers); result != 1)
    {
        throw std::runtime_error("Invalid number of readers: '"
            + std::string(readers_str) + "'");
    }

    if(readers <= 0)
    {
        throw std::runtime_error("Number of readers in the channel must be grater than zero, got '"
            + std::string(readers_str) + "'");
    }

    return readers;
}

int main(int argc, char** argv)
{
    if(argc < 4)
    {
        print_usage();
        return EXIT_FAILURE;
    }

    // Get arguments from command line
    const auto name_str = argv[1];
    const auto size_str = argv[2];
    const auto readers_str = argv[3];

    try
    {
        // Parse arguments
        const auto name = parse_name(name_str);
        const auto size = parse_size(size_str);
        const auto readers = parse_readers(readers_str);

        // Initialize ROScpp in order to be gracefully killed via signals
        ros::init(argc, argv, "shm_server");

        // Create channel
        printf("[shm_server] Creating channel '%s' with size %d and %d readers...\n", name, size, readers);
        const auto channel = shm::Channel(name, size, readers);

        // Wait for exit
        printf("[shm_server] Server initialized! Waiting for signal to exit...\n");
        ros::spin();

        printf("[shm_server] Signal occured. Closing server...\n");
    }
    catch(std::exception& ex)
    {
        printf("Error: %s\n", ex.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS; // Channel instance will be deleted with destructor automatically
}
