///////////////////////////////////////////////////////////////////////////////
// main.cpp
//
// Contains implementation of the entry point for shm_server application
///////////////////////////////////////////////////////////////////////////////

#include <cassert>
#include <cstdio>

#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <ros/ros.h>

#include "shm_comm/Channel.hpp"

/**
 * @brief Helper structure, defines shared memory channel configuration
 */
struct ChannelConfig
{
    std::string name;
    int size;
    int readers;
};

//! Helper typedef, arranges many shm channel configs
using ChannelsConfigs = std::vector<ChannelConfig>;

static void print_usage()
{
    printf("Usage: ./shm_server <config_path>\n");
    printf(" where\n");
    printf("  - <config_path> - path to JSON file with configuration of server\n");
}

static std::string parse_name(const nlohmann::json& json)
{
    const auto name = json.at("name").get<std::string>();
    if(name.size() == 0 || name.size() >= 128)
    {
        throw std::runtime_error("Invalid channel name size. Must be less than 128 characters");
    }

    return name;
}

static int parse_size(const nlohmann::json& json)
{
    const auto size = json.at("size").get<int>();
    if(size <= 0)
    {
        throw std::runtime_error("Invalid channel size. Must be greater than zero");
    }

    return size;
}

static int parse_readers(const nlohmann::json& json)
{
    const auto readers = json.at("readers").get<int>();
    if(readers <= 0)
    {
        throw std::runtime_error("Invalid number of channel readers. Must be greater than zero");
    }

    return readers;
}

static void from_json(const nlohmann::json& json, ChannelConfig& channel_config)
{
    channel_config.name = parse_name(json);
    channel_config.size = parse_size(json);
    channel_config.readers = parse_readers(json);
}

static nlohmann::json read_config(const char* path)
{
    assert(path != nullptr);

    // Open configuration file
    std::ifstream ifs(path);
    if(!ifs)
    {
        throw std::runtime_error("Could not open configuration file: "
            + std::string(path));
    }

    // Parse configuration from JSON
    auto json = nlohmann::json();
    if(!(ifs >> json))
    {
        throw std::runtime_error("Could not read configuration file: "
            + std::string(path));
    }

    return json;
}

static ChannelsConfigs read_channels_configs(const char* path)
{
    const auto json = read_config(path);
    const auto channels_json = json.at("channels");
    return channels_json.get<ChannelsConfigs>();
}

/**
 * @brief Entry point to the application
 * @details Reads configuration of the server and initializes shared memory channels
 * It gets path to server configuration JSON from command line,
 *  initializes ROS client library, reads configuration JSON, parses
 *  channels' condfigurations and initializes them.
 * After successfull initialization it waits for interruption via signal,
 *  after which it removes created channels.
 */
int main(int argc, char** argv)
{
    if(argc < 2)
    {
        print_usage();
        return EXIT_FAILURE;
    }

    try
    {
        printf("[shm_server] Initializing ROS client...\n");
        ros::init(argc, argv, "shm_server");

        printf("[shm_server] Reading channels configuration...\n");
        const auto config_path = argv[1];
        const auto channels_configs = read_channels_configs(config_path);

        printf("[shm_server] Initializing shared memory channels...\n");
        std::vector<shm::Channel> channels;
        channels.reserve(channels_configs.size());
        for(const auto& [name, size, readers] : channels_configs)
        {
            channels.emplace_back(name.c_str(), size, readers);
        }

        printf("[shm_server] Server initialized! Waiting for signal to exit...\n");
        ros::spin();
    }
    catch(std::exception& ex)
    {
        printf("Error: %s\n", ex.what());
        return EXIT_FAILURE;
    }

    printf("[shm_server] Server deinitialized\n");
    return EXIT_SUCCESS; // Channel instance will be deleted with destructor automatically
}
